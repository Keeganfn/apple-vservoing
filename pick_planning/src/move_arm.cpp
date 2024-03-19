#include <rclcpp/rclcpp.hpp>

// Include our custom message type definition.
#include "pick_planning_interfaces/srv/move_arm.hpp"
#include "pick_planning_interfaces/srv/move_to_named_target.hpp"

// Moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

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
    rclcpp::Service<pick_planning_interfaces::srv::MoveToNamedTarget>::SharedPtr arm_named_service_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_; 
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber2_;
    void move_to_pose(const std::shared_ptr<pick_planning_interfaces::srv::MoveArm::Request> request,
		              const std::shared_ptr<pick_planning_interfaces::srv::MoveArm::Response> response);
    void move_to_named_target(const std::shared_ptr<pick_planning_interfaces::srv::MoveToNamedTarget::Request> request,
                              const std::shared_ptr<pick_planning_interfaces::srv::MoveToNamedTarget::Response> response);
};


MoveArmNode::MoveArmNode() : Node("move_arm_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)), 
                                move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), "ur_manipulator")
{
    // callback groups
    callback_group_subscriber1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_subscriber2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    

    // Set up services, one for pose goals and one for named target goals
    arm_service_ = this->create_service<pick_planning_interfaces::srv::MoveArm>("move_arm", std::bind(&MoveArmNode::move_to_pose, this, _1, _2), 
                                                                                rmw_qos_profile_services_default, callback_group_subscriber1_);
    arm_named_service_ = this->create_service<pick_planning_interfaces::srv::MoveToNamedTarget>("move_arm_named_target", std::bind(&MoveArmNode::move_to_named_target, this, _1, _2),
                                                                                                 rmw_qos_profile_services_default, callback_group_subscriber2_);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    auto const collision_object = [frame_id =move_group_.getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "box1";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 1;
        primitive.dimensions[primitive.BOX_Y] = 1;
        primitive.dimensions[primitive.BOX_Z] = 0.015;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
        box_pose.position.x = 0.0;
        box_pose.position.y = 0.0;
        box_pose.position.z = -0.01;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }();
    planning_scene_interface_.applyCollisionObject(collision_object);
    // vel and acc limits
    this->move_group_.setMaxAccelerationScalingFactor(1.0);
    this->move_group_.setMaxVelocityScalingFactor(1.0);
    RCLCPP_INFO(this->get_logger(), "Move arm server ready");
}

void MoveArmNode::move_to_named_target(const std::shared_ptr<pick_planning_interfaces::srv::MoveToNamedTarget::Request> request,
                                       const std::shared_ptr<pick_planning_interfaces::srv::MoveToNamedTarget::Response> response)
{
    // Attempts to move to named target goal 
    if (!this->move_group_.setNamedTarget(request->name)) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Could not find target " << request->name);
        response->result = false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan goal;
    auto const ok = static_cast<bool>(move_group_.plan(goal));
    
    if (ok){
        this->move_group_.execute(goal);
        response->result = true;
    }
    else{
        RCLCPP_ERROR(this->get_logger(), "Planing failed!");
        response->result = false;
    }

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

    // Attempts to move to pose goal
    moveit::planning_interface::MoveGroupInterface::Plan goal;
    auto const ok = static_cast<bool>(move_group_.plan(goal));
    response->result = true;
    if (ok){
        this->move_group_.execute(goal);
    }
    else{
        RCLCPP_ERROR(this->get_logger(), "Planing failed!");
        response->result = false;
    }

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

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(move_service);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}