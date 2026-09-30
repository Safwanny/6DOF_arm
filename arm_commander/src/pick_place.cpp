#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

namespace mtc = moveit::task_constructor;

const std::string ARM = "arm";
const std::string HAND = "gripper";
const std::string HAND_FRAME = "tool_link";

// Scene layout (base_link frame, metres). Tune these if the task fails to plan.
const double TABLE_TOP = 0.15;
const double OBJECT_SIZE[3] = {0.04, 0.04, 0.10};
const double PICK_XY[2] = {0.6, -0.2};
const double PLACE_XY[2] = {0.6, 0.2};
// Object centre along tool_link z when grasped: fingers span 0.02-0.10 m past tool_link
const double GRASP_DEPTH = 0.075;
// SRDF gripper state that closes just onto the box (fully closing would crush it in Gazebo)
const std::string GRASP_STATE = "gripper_grasp";

moveit_msgs::msg::CollisionObject makeBox(const std::string &id, double sx, double sy, double sz,
                                          double x, double y, double z)
{
    moveit_msgs::msg::CollisionObject obj;
    obj.id = id;
    obj.header.frame_id = "base_link";
    obj.primitives.resize(1);
    obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    obj.primitives[0].dimensions = {sx, sy, sz};
    obj.pose.position.x = x;
    obj.pose.position.y = y;
    obj.pose.position.z = z;
    obj.pose.orientation.w = 1.0;
    obj.operation = obj.ADD;
    return obj;
}

void setupPlanningScene()
{
    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObjects({
        makeBox("table", 0.4, 0.8, TABLE_TOP, 0.65, 0.0, TABLE_TOP / 2),
        makeBox("object", OBJECT_SIZE[0], OBJECT_SIZE[1], OBJECT_SIZE[2],
                PICK_XY[0], PICK_XY[1], TABLE_TOP + OBJECT_SIZE[2] / 2),
    });
}

mtc::Task createTask(const rclcpp::Node::SharedPtr &node)
{
    mtc::Task task;
    task.stages()->setName("pick and place");
    task.loadRobotModel(node);
    task.setProperty("group", ARM);
    task.setProperty("eef", HAND);
    task.setProperty("ik_frame", HAND_FRAME);

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node);
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setStepSize(0.01);

    auto hand_links = task.getRobotModel()->getJointModelGroup(HAND)->getLinkModelNamesWithCollisionGeometry();

    geometry_msgs::msg::Vector3Stamped world_up;
    world_up.header.frame_id = "base_link";
    world_up.vector.z = 1.0;

    auto current = std::make_unique<mtc::stages::CurrentState>("current");
    mtc::Stage *current_ptr = current.get();
    task.add(std::move(current));

    auto open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    open_hand->setGroup(HAND);
    open_hand->setGoal("gripper_open");
    task.add(std::move(open_hand));

    auto move_to_pick = std::make_unique<mtc::stages::Connect>(
        "move to pick", mtc::stages::Connect::GroupPlannerVector{{ARM, sampling_planner}});
    move_to_pick->setTimeout(5.0);
    move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(move_to_pick));

    mtc::Stage *attach_ptr = nullptr;
    {
        auto pick = std::make_unique<mtc::SerialContainer>("pick object");
        task.properties().exposeTo(pick->properties(), {"eef", "group", "ik_frame"});
        pick->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

        // Come straight down onto the object along the tool axis
        auto approach = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
        approach->properties().set("link", HAND_FRAME);
        approach->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        approach->setMinMaxDistance(0.08, 0.15);
        geometry_msgs::msg::Vector3Stamped tool_forward;
        tool_forward.header.frame_id = HAND_FRAME;
        tool_forward.vector.z = 1.0;
        approach->setDirection(tool_forward);
        pick->insert(std::move(approach));

        // Sample grasps around the object's vertical axis, tool pointing down.
        // 90 deg steps keep the fingers flat on the box faces; a diagonal grasp pinches the
        // corners (5.7 cm across) and squirts the box out in Gazebo.
        auto grasp = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
        grasp->properties().configureInitFrom(mtc::Stage::PARENT);
        grasp->setPreGraspPose("gripper_open");
        grasp->setObject("object");
        grasp->setAngleDelta(M_PI / 2);
        grasp->setMonitoredStage(current_ptr);

        Eigen::Isometry3d grasp_frame = Eigen::Isometry3d::Identity();
        grasp_frame.translation().z() = GRASP_DEPTH;
        grasp_frame.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

        auto grasp_ik = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(grasp));
        grasp_ik->setMaxIKSolutions(8);
        grasp_ik->setMinSolutionDistance(1.0);
        grasp_ik->setIKFrame(grasp_frame, HAND_FRAME);
        grasp_ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
        grasp_ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
        pick->insert(std::move(grasp_ik));

        auto allow = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
        allow->allowCollisions("object", hand_links, true);
        pick->insert(std::move(allow));

        auto close_hand = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
        close_hand->setGroup(HAND);
        close_hand->setGoal(GRASP_STATE);
        pick->insert(std::move(close_hand));

        auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
        attach->attachObject("object", HAND_FRAME);
        attach_ptr = attach.get();
        pick->insert(std::move(attach));

        auto lift = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
        lift->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        lift->setMinMaxDistance(0.05, 0.2);
        lift->setIKFrame(HAND_FRAME);
        lift->setDirection(world_up);
        pick->insert(std::move(lift));

        task.add(std::move(pick));
    }

    // Arm only: the hand stays closed, and no SRDF group spans arm + hand for a merged trajectory
    auto move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place", mtc::stages::Connect::GroupPlannerVector{{ARM, sampling_planner}});
    move_to_place->setTimeout(5.0);
    move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(move_to_place));

    {
        auto place = std::make_unique<mtc::SerialContainer>("place object");
        task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
        place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

        auto place_pose = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
        place_pose->properties().configureInitFrom(mtc::Stage::PARENT);
        place_pose->setObject("object");
        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = "base_link";
        target.pose.position.x = PLACE_XY[0];
        target.pose.position.y = PLACE_XY[1];
        target.pose.position.z = TABLE_TOP + OBJECT_SIZE[2] / 2;
        target.pose.orientation.w = 1.0;
        place_pose->setPose(target);
        place_pose->setMonitoredStage(attach_ptr);

        auto place_ik = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(place_pose));
        place_ik->setMaxIKSolutions(4);
        place_ik->setMinSolutionDistance(1.0);
        place_ik->setIKFrame("object");
        place_ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
        place_ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
        place->insert(std::move(place_ik));

        auto open = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
        open->setGroup(HAND);
        open->setGoal("gripper_open");
        place->insert(std::move(open));

        auto forbid = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
        forbid->allowCollisions("object", hand_links, false);
        place->insert(std::move(forbid));

        auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
        detach->detachObject("object", HAND_FRAME);
        place->insert(std::move(detach));

        auto retreat = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
        retreat->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        retreat->setMinMaxDistance(0.05, 0.2);
        retreat->setIKFrame(HAND_FRAME);
        retreat->setDirection(world_up);
        place->insert(std::move(retreat));

        task.add(std::move(place));
    }

    auto home = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
    home->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    home->setGoal("home");
    task.add(std::move(home));

    return task;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("pick_place", options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    setupPlanningScene();
    auto task = createTask(node);

    try {
        task.init();
    }
    catch (mtc::InitStageException &e) {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Task init failed: " << e);
        rclcpp::shutdown();
        spinner.join();
        return 1;
    }

    if (!task.plan(5) || task.solutions().empty()) {
        RCLCPP_ERROR(node->get_logger(), "Task planning failed, check the stage that has 0 solutions");
    }
    else {
        task.introspection().publishSolution(*task.solutions().front());
        auto result = task.execute(*task.solutions().front());
        if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Pick and place done");
        }
        else {
            RCLCPP_ERROR(node->get_logger(), "Task execution failed with code %d", result.val);
        }
    }

    // Keep the node alive so the solution stays visible in RViz's Motion Planning Tasks panel
    spinner.join();
    return 0;
}
