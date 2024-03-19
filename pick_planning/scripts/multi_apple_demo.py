#!/usr/bin/env python3

# ROS
import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup,  MutuallyExclusiveCallbackGroup
# Interfaces
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, TwistStamped
from pick_planning_interfaces.srv import MoveArm, StartSequence, MoveToNamedTarget, GlobalPlannerSequence
from controller_manager_msgs.srv import SwitchController
# Image processing
from cv_bridge import CvBridge
import cv2
import numpy as np
# TF2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import tf2_geometry_msgs
# Gazebo
from gazebo_msgs.srv import DeleteEntity
# Python
import sys

class MultiAppleDemo(Node):
    def __init__(self, name):
        super().__init__("global_planner")
        self.get_logger().info(f"Starting {name}...")
        ### Services 
        m_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Global planner
        self.global_sequence_client= self.create_client(GlobalPlannerSequence, "/start_global_sequence", callback_group=m_callback_group)
        while not self.global_sequence_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Global planner service not available, waiting...")
        # Local planner
        self.local_sequence_client= self.create_client(StartSequence, "/start_local_sequence", callback_group=m_callback_group)
        while not self.local_sequence_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Local planner service not available, waiting...")
        # Moveit move to named target service
        self.cpp_move_named_client= self.create_client(MoveToNamedTarget, "/move_arm_named_target", callback_group=m_callback_group)
        while not self.local_sequence_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Move arm to named target service not available, waiting...")
        # Client for switching controller to joint_trajcectory_controller
        self.switch_controller_client = self.create_client(SwitchController, "/controller_manager/switch_controller", callback_group=m_callback_group)
        while not self.switch_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Switch controller service not available, waiting...")
        # Client for C++ moveit node to execute pose goals 
        self.cpp_arm_client = self.create_client(MoveArm, "/move_arm",callback_group=m_callback_group)
        while not self.cpp_arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Move arm service not available, waiting...")
        # Service to activate servo mode on arm
        self.start_servo_client = self.create_client(Trigger, "/servo_node/start_servo", callback_group=m_callback_group)
        while not self.start_servo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Start servo service not available, waiting...")
        self.start_servo()
        # Despawn entity gazebo service
        self.gazebo_delete_client = self.create_client(DeleteEntity, "/delete_entity", callback_group=m_callback_group)
        while not self.gazebo_delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Gazebo delete entity service not available, waiting...")

        ### Apple names
        self.apple_names = ["apple1_1_clone", "apple1_1", "apple1_1_clone_0", "apple1_1_clone_0_clone", "apple1_1_clone_0_clone_clone"]

    def delete_apple(self):
        # Starts global planning sequence
        self.request = DeleteEntity.Request()
        self.request.name = self.apple_names.pop(0)
        self.future = self.gazebo_delete_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def start_global(self):
        # Starts global planning sequence
        self.request = GlobalPlannerSequence.Request()
        self.future = self.global_sequence_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().goal
    
    def start_local(self):
        # Starts global planning sequence
        self.request = StartSequence.Request()
        self.future = self.local_sequence_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def reset_to_target(self):
        # Starts global planning sequence
        self.request = MoveToNamedTarget.Request()
        self.request.name = "start_config"
        self.future = self.cpp_move_named_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def switch_controller(self, servo=False):
        # Switches controller from forward position controller to joint_trajectory controller
        self.request = SwitchController.Request()
        if servo:
            self.request.activate_controllers = ["forward_position_controller"] 
            self.request.deactivate_controllers = ["joint_trajectory_controller"]
        else:
            self.request.activate_controllers = ["joint_trajectory_controller"]
            self.request.deactivate_controllers = ["forward_position_controller"]
        self.request.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()
        self.future = self.switch_controller_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_pose_goal(self, pose):
        print(pose)
        # Sends x,y,z to C++ moveit node to execute pose goal since python moveit not available for humble
        self.request = MoveArm.Request()
        self.request.goal.x = pose.x
        self.request.goal.y = pose.y
        self.request.goal.z = pose.z
        self.future = self.cpp_arm_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def start_servo(self):
        # Starts servo node
        self.request = Trigger.Request()
        self.future = self.start_servo_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def demo_full(self):
        for i in range(len(self.apple_names)):
            ### GLOBAL
            self.get_logger().info(f"Starting global planner sequence...")
            self.get_logger().info(f"Switching to joint_trajectory_controller...")
            #switch controller and get next apple goal from global planner
            self.switch_controller()
            goal = self.start_global()
            self.get_logger().info(f"Sending pose goal to moveit node...")
            # execute initial movement by sending pose goal to moveit c++ node
            result = self.send_pose_goal(goal)

            ### LOCAL
            self.get_logger().info(f"Starting local planner sequence...")
            self.get_logger().info(f"Switching to forward_position_controller...")
            self.switch_controller(servo=True)
            self.start_servo()
            self.start_local()
            self.get_logger().info(f"Picking apple...")
            self.delete_apple()

            ### RESET
            self.get_logger().info(f"Resetting arm to initial position...")
            self.switch_controller()
            self.reset_to_target()
        self.get_logger().info(f"All apples picked, demo complete!")

    def demo_local(self):
        for i in range(len(self.apple_names)):
            ### LOCAL
            self.get_logger().info(f"Starting local planner sequence...")
            self.get_logger().info(f"Switching to forward_position_controller...")
            self.switch_controller(servo=True)
            self.start_servo()
            self.start_local()
            self.get_logger().info(f"Picking apple...")
            self.delete_apple()

            ### RESET
            self.get_logger().info(f"Resetting arm to initial position...")
            self.switch_controller()
            self.reset_to_target()
        self.get_logger().info(f"All apples picked, demo complete!")


def main(args=None):
    rclpy.init(args=args)
    demo_name = str(sys.argv[1])
    demo = MultiAppleDemo(demo_name)
    if demo_name == "demo1":
        demo.demo_full()
    else:
        demo.demo_local()
    executor = MultiThreadedExecutor()
    rclpy.spin(demo, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    print(sys.argv)
    main()