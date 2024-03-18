#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

// Servo
// #include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char ** argv)
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // Set a target Pose
  auto const target_pose = [] {
    tf2::Quaternion orientation;
    orientation.setRPY(3.14/2, 3.14, 3.14/2);
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "base_link";
    msg.pose.orientation = tf2::toMsg(orientation);;
    msg.pose.position.x = 0.35;
    msg.pose.position.y = 0;
    msg.pose.position.z = 0.6;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose, "tool0");
  move_group_interface.setGoalOrientationTolerance(.1);
  // Create a plan to that target pose

  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  // Execute the plan
  if (ok)
  {
    move_group_interface.execute(msg);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;

}
