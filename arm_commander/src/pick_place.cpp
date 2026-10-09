#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <sstream>
#include <arm_interfaces/srv/pick_place.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <std_srvs/srv/trigger.hpp>

// Runs one MTC task per service call; packer.py (Python) decides which cube goes where.
namespace mtc = moveit::task_constructor;

const std::string ARM = "arm";
const std::string HAND = "gripper";
const std::string HAND_FRAME = "tool_link";

// Scene layout (base_link frame, metres), see arm_bringup/config/scene.yaml; packer.py sets up the planning scene
const double TABLE_TOP = 0.15;
const double OBJECT_SIZE[3] = {0.04, 0.04, 0.04};
const double TRAY_FLOOR = 0.005;  // thickness, see scene.yaml
// Release this far above the tray floor so the box settles instead of being pushed into it
const double RELEASE_GAP = 0.003;
// Object centre along tool_link z when grasped: fingers span 0.02-0.10 m past tool_link,
// so the tips stop 1.5 cm above the cube bottom (clear of the table and the 1 cm tray walls)
const double GRASP_DEPTH = 0.095;
// SRDF gripper state aimed 1 cm inside the cube; in Gazebo the force-controlled fingers stop on it
const std::string GRASP_STATE = "gripper_grasp";

mtc::Task createTask(const rclcpp::Node::SharedPtr &node, const std::string &object, double x, double y)
{
    mtc::Task task;
    task.stages()->setName("pick and place " + object);
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
        grasp->setObject(object);
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
        allow->allowCollisions(object, hand_links, true);
        pick->insert(std::move(allow));

        auto close_hand = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
        close_hand->setGroup(HAND);
        close_hand->setGoal(GRASP_STATE);
        pick->insert(std::move(close_hand));

        auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
        attach->attachObject(object, HAND_FRAME);
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

        // Come straight down into the slot so the box never swings over the tray walls
        auto lower = std::make_unique<mtc::stages::MoveRelative>("lower object", cartesian_planner);
        lower->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        lower->setMinMaxDistance(0.03, 0.1);
        lower->setIKFrame(HAND_FRAME);
        geometry_msgs::msg::Vector3Stamped world_down = world_up;
        world_down.vector.z = -1.0;
        lower->setDirection(world_down);
        place->insert(std::move(lower));

        auto place_pose = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
        place_pose->properties().configureInitFrom(mtc::Stage::PARENT);
        place_pose->setObject(object);
        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = "base_link";
        target.pose.position.x = x;
        target.pose.position.y = y;
        target.pose.position.z = TABLE_TOP + TRAY_FLOOR + OBJECT_SIZE[2] / 2 + RELEASE_GAP;
        target.pose.orientation.w = 1.0;
        place_pose->setPose(target);
        place_pose->setMonitoredStage(attach_ptr);

        auto place_ik = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(place_pose));
        place_ik->setMaxIKSolutions(4);
        place_ik->setMinSolutionDistance(1.0);
        place_ik->setIKFrame(object);
        place_ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
        place_ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
        place->insert(std::move(place_ik));

        // Half open only: fully open fingers would reach into the neighbouring slots
        auto open = std::make_unique<mtc::stages::MoveTo>("release", interpolation_planner);
        open->setGroup(HAND);
        open->setGoal("gripper_half_open");
        place->insert(std::move(open));

        auto forbid = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
        forbid->allowCollisions(object, hand_links, false);
        place->insert(std::move(forbid));

        auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
        detach->detachObject(object, HAND_FRAME);
        place->insert(std::move(detach));

        auto retreat = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
        retreat->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        retreat->setMinMaxDistance(0.05, 0.2);
        retreat->setIKFrame(HAND_FRAME);
        retreat->setDirection(world_up);
        place->insert(std::move(retreat));

        task.add(std::move(place));
    }

    // Wait above the table for the next cube (SRDF "ready"); packer.py sends the arm home once every cube is packed
    auto ready = std::make_unique<mtc::stages::MoveTo>("move to ready", sampling_planner);
    ready->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    ready->setGoal("ready");
    task.add(std::move(ready));

    return task;
}

bool runTask(const rclcpp::Node::SharedPtr &node, mtc::Task &task)
{
    try {
        task.init();
    }
    catch (mtc::InitStageException &e) {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Task init failed: " << e);
        return false;
    }
    if (!task.plan(5) || task.solutions().empty()) {
        std::ostringstream why;
        task.explainFailure(why);
        RCLCPP_ERROR(node->get_logger(), "Planning %s failed:\n%s", task.name().c_str(), why.str().c_str());
        return false;
    }
    task.introspection().publishSolution(*task.solutions().front());
    auto result = task.execute(*task.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Executing %s failed with code %d", task.name().c_str(), result.val);
        return false;
    }
    return true;
}

mtc::Task createHomeTask(const rclcpp::Node::SharedPtr &node)
{
    mtc::Task task;
    task.stages()->setName("go home");
    task.loadRobotModel(node);
    task.setProperty("group", ARM);
    task.add(std::make_unique<mtc::stages::CurrentState>("current"));
    auto home = std::make_unique<mtc::stages::MoveTo>("return home", std::make_shared<mtc::solvers::PipelinePlanner>(node));
    home->setGroup(ARM);
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

    // A service call blocks while its task runs, and MTC still needs the node spinning to execute:
    // services get their own group on a multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // Tasks stay alive so every solution remains browsable in RViz's Motion Planning Tasks panel
    std::vector<std::unique_ptr<mtc::Task>> tasks;
    auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto pick_place = node->create_service<arm_interfaces::srv::PickPlace>(
        "pick_place",
        [&](const std::shared_ptr<arm_interfaces::srv::PickPlace::Request> req,
            std::shared_ptr<arm_interfaces::srv::PickPlace::Response> res) {
            tasks.push_back(std::make_unique<mtc::Task>(createTask(node, req->object, req->x, req->y)));
            res->success = runTask(node, *tasks.back());
        },
        rclcpp::ServicesQoS(), group);
    auto go_home = node->create_service<std_srvs::srv::Trigger>(
        "go_home",
        [&](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            tasks.push_back(std::make_unique<mtc::Task>(createHomeTask(node)));
            res->success = runTask(node, *tasks.back());
        },
        rclcpp::ServicesQoS(), group);
    RCLCPP_INFO(node->get_logger(), "Ready for /pick_place and /go_home");

    spinner.join();
    return 0;
}
