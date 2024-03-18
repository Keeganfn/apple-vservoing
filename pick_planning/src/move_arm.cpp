#include <rclcpp/rclcpp.hpp>

// Include our custom message type definition.
#include "pick_planning_interfaces/srv/move_arm.hpp"

// Moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// The service callback is going to need two parameters, so we declare that
// we're going to use two placeholders.
using std::placeholders::_1;
using std::placeholders::_2;


// Create a class that inherits from Node.
class MoveArmNode : public rclcpp::Node {
public:
	MoveArmNode();
    // Move group interface
    moveit::planning_interface::MoveGroupInterface move_group_;
    void test_arm();

private:
    rclcpp::Service<pick_planning_interfaces::srv::MoveArm>::SharedPtr arm_service_;
    void move_to_pose(const std::shared_ptr<pick_planning_interfaces::srv::MoveArm::Request> request,
		                       const std::shared_ptr<pick_planning_interfaces::srv::MoveArm::Response> response);
};


MoveArmNode::MoveArmNode() : Node("move_arm_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)), 
                                move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), "ur_manipulator")
{
    // Set up a service server.  This callback takes two arguments, so we use two placeholders.
    arm_service_ = this->create_service<pick_planning_interfaces::srv::MoveArm>("move_arm", std::bind(&MoveArmNode::move_to_pose, this, _1, _2));
    // vel and acc limits
    this->move_group_.setMaxAccelerationScalingFactor(1.0);
    this->move_group_.setMaxVelocityScalingFactor(1.0);
    RCLCPP_INFO(this->get_logger(), "Move arm server ready");
    this->test_arm();
}


void MoveArmNode::move_to_pose(const std::shared_ptr<pick_planning_interfaces::srv::MoveArm::Request> request,
                        const std::shared_ptr<pick_planning_interfaces::srv::MoveArm::Response> response)
{
    // Make posestamped message
    tf2::Quaternion orientation;
    orientation.setRPY(3.14/2, 3.14, 3.14/2);
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "base_link";
    msg.pose.orientation = tf2::toMsg(orientation);;
    msg.pose.position.x = request->goal.x;
    msg.pose.position.y = request->goal.y;
    msg.pose.position.z = request->goal.z;
    this->move_group_.setPoseTarget(msg, "tool0");
    this->move_group_.setGoalOrientationTolerance(.1);

    moveit::planning_interface::MoveGroupInterface::Plan goal;
    auto const ok = static_cast<bool>(move_group_.plan(goal));
    if (ok){
        this->move_group_.execute(goal);
    }
    else{
        RCLCPP_ERROR(this->get_logger(), "Planing failed!");
    }

    response->result = true;
}

void MoveArmNode::test_arm()
{
    tf2::Quaternion orientation;
    orientation.setRPY(3.14/2, 3.14, 3.14/2);
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "base_link";
    msg.pose.orientation = tf2::toMsg(orientation);;
    msg.pose.position.x = 0.35;
    msg.pose.position.y = 0;
    msg.pose.position.z = 0.6;
    this->move_group_.setPoseTarget(msg, "tool0");
    this->move_group_.setGoalOrientationTolerance(.1);

    moveit::planning_interface::MoveGroupInterface::Plan goal;
    auto const ok = static_cast<bool>(move_group_.plan(goal));
    if (ok){
        this->move_group_.execute(goal);
    }
    else{
        RCLCPP_ERROR(this->get_logger(), "Planing failed!");
    }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto move_service = std::make_shared<MoveArmNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_service);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}