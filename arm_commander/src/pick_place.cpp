#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>
#include <arm_interfaces/srv/pick_place.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <std_srvs/srv/trigger.hpp>

// Runs one MTC task per service call; packer.py (Python) decides which box goes where.
namespace mtc = moveit::task_constructor;

const std::string ARM = "arm";
const std::string HAND = "gripper";
const std::string HAND_FRAME = "tool_link";

// Gripper geometry (gripper.xacro): finger inner faces are OPEN_GAP apart at joint value 0 and each
// finger closes by its joint value; the fingers span 0.02-0.10 m past tool_link.
const double OPEN_GAP = 0.12;
const double FINGER_END = 0.10;
const std::vector<std::string> FINGERS = {"gripper_left_finger_joint", "gripper_right_finger_joint"};
// Finger tips stop this far above the box bottom: clear of the surface below and the 1.5 cm tray walls
const double TIP_CLEARANCE = 0.015;
// Aim this far inside each side of the box; in Gazebo the force-controlled fingers stop on it
const double SQUEEZE = 0.01;
// Open this far clear of each side to release, so the fingers don't reach into neighbouring boxes
const double RELEASE_CLEARANCE = 0.015;
// Release this far above the surface so the box settles instead of being pushed into it
const double RELEASE_GAP = 0.003;

// Both fingers at the joint value that leaves `gap` between them
std::map<std::string, double> fingersAt(double gap)
{
    const double q = std::clamp((OPEN_GAP - gap) / 2, 0.0, 0.06);
    return {{FINGERS[0], q}, {FINGERS[1], q}};
}

mtc::Task createTask(const rclcpp::Node::SharedPtr &node, const arm_interfaces::srv::PickPlace::Request &req)
{
    const std::string &object = req.object;
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

        // Grasp across the object's x axis (the gripped side), tool pointing down: 180 deg steps give the
        // two hand orientations that keep the fingers flat on those faces.
        auto grasp = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
        grasp->properties().configureInitFrom(mtc::Stage::PARENT);
        grasp->setPreGraspPose("gripper_open");
        grasp->setObject(object);
        grasp->setAngleDelta(M_PI);
        grasp->setMonitoredStage(current_ptr);

        Eigen::Isometry3d grasp_frame = Eigen::Isometry3d::Identity();
        // Object centre along tool z: tips TIP_CLEARANCE above the box bottom
        grasp_frame.translation().z() = FINGER_END + TIP_CLEARANCE - req.height / 2;
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
        close_hand->setGoal(fingersAt(req.grip_width - 2 * SQUEEZE));
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

        // Come straight down onto the spot so the box never swings over the tray walls or other boxes
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
        target.pose.position.x = req.x;
        target.pose.position.y = req.y;
        target.pose.position.z = req.z + req.height / 2 + RELEASE_GAP;
        target.pose.orientation.z = std::sin(req.yaw / 2);
        target.pose.orientation.w = std::cos(req.yaw / 2);
        place_pose->setPose(target);
        place_pose->setMonitoredStage(attach_ptr);

        auto place_ik = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(place_pose));
        place_ik->setMaxIKSolutions(4);
        place_ik->setMinSolutionDistance(1.0);
        place_ik->setIKFrame(object);
        place_ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
        place_ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
        place->insert(std::move(place_ik));

        // Open just clear of the box: fully open fingers would reach into the neighbouring boxes
        auto open = std::make_unique<mtc::stages::MoveTo>("release", interpolation_planner);
        open->setGroup(HAND);
        open->setGoal(fingersAt(req.grip_width + 2 * RELEASE_CLEARANCE));
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

    // Wait above the table for the next box (SRDF "ready"); packer.py sends the arm home once every box is packed
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
            tasks.push_back(std::make_unique<mtc::Task>(createTask(node, *req)));
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
