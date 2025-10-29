#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);

    auto gripper = moveit::planning_interface::MoveGroupInterface(node, "gripper");

    // //NAMED GOALS
    // //  arm = current_state -> pose1
    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("pose1");

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success1) {
    //     arm.execute(plan1);
    // }

    // // gripper = current position -> half closed
    // gripper.setStartStateToCurrentState();
    // gripper.setNamedTarget("gripper_half_open");

    // moveit::planning_interface::MoveGroupInterface::Plan plan2;
    // bool success2 = (gripper.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success2) {
    //     gripper.execute(plan2);
    // }

    // // arm = current_state -> pose2
    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("pose2");

    // moveit::planning_interface::MoveGroupInterface::Plan plan3;
    // bool success3 = (arm.plan(plan3) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success3) {
    //     arm.execute(plan3);
    // }

    // // gripper = current position -> open
    // gripper.setStartStateToCurrentState();
    // gripper.setNamedTarget("gripper_open");

    // moveit::planning_interface::MoveGroupInterface::Plan plan4;
    // bool success4 = (gripper.plan(plan4) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success4) {
    //     gripper.execute(plan4);
    // }

    // // arm = current_state -> home
    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("home");

    // moveit::planning_interface::MoveGroupInterface::Plan plan5;
    // bool success5 = (arm.plan(plan5) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success5) {
    //     arm.execute(plan5);
    // }

    //-------------------------------------------------------------------------------------------

    // // JOINT GOALS
    // // arm = current state -> joints
    // std::vector<double> joints = { 1.5, 0.5, 0.0, 1.5, 0.0, -0.7 };

    // arm.setStartStateToCurrentState();
    // arm.setJointValueTarget(joints);

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success1) {
    //     arm.execute(plan1);
    // }

    //-------------------------------------------------------------------------------------------

    // POSE GOALS

    // arm = current state -> target_pose
    tf2::Quaternion q;
    q.setRPY(3.14, 0.0, 0.0);
    q = q.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = 0.7;
    target_pose.pose.position.y = 0.0;
    target_pose.pose.position.z = 0.4;
    target_pose.pose.orientation.x = q.getX();
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success1) {
        arm.execute(plan1);
    }

    //-------------------------------------------------------------------------------------------

    // CARTESIAN PATHS

    // use with pose goals (they are connected)
    // movement of robot in cartesian motions
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose pose1 = arm.getCurrentPose().pose;
    pose1.position.z += -0.2;
    waypoints.push_back(pose1);
    geometry_msgs::msg::Pose pose2 = pose1;
    pose2.position.y += 0.2;
    waypoints.push_back(pose2); 
    geometry_msgs::msg::Pose pose3 = pose2;
    pose3.position.y += -0.2;
    pose3.position.z += 0.2;
    waypoints.push_back(pose3);

    moveit_msgs::msg::RobotTrajectory trajectory;

    double fraction = arm.computeCartesianPath(waypoints, 0.01, trajectory);

    if (fraction == 1) {
        arm.execute(trajectory);
    }



    rclcpp::shutdown();
    spinner.join();
    return 0;
}