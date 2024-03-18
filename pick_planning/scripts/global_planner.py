#!/usr/bin/env python3

# ROS
import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
# Interfaces
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from pick_planning_interfaces.srv import MoveArm, StartGlobalSequence
from controller_manager_msgs.srv import SwitchController
# Image processing
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge
import cv2
import numpy as np
# TF2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import tf2_geometry_msgs

class GlobalPlanner(Node):

    def __init__(self):
        super().__init__("global_planner")
        ### SUBSCRIBERS
        # Depth, rgb and camera info data
        self.image_sub= Subscriber(self,Image,"/camera2/image_raw")
        self.depth_sub = Subscriber(self,Image,"/camera2/depth/image_raw")
        self.camera_info_sub = Subscriber(self,CameraInfo,"/camera2/camera_info")
        #specify reentrant callback group 
        r_callback_group = ReentrantCallbackGroup()
        # Time synchronizer for depth ,RGB and camera info image feed
        self.ts = ApproximateTimeSynchronizer([self.image_sub, self.depth_sub, self.camera_info_sub],30,0.01,)
        self.ts.registerCallback(self.rgbd_callback)

        ### SERVICES
        # Client for switching controller to joint_trajcectory_controller
        self.switch_controller_client = self.create_client(SwitchController, "/controller_manager/switch_controller", callback_group=r_callback_group)
        while not self.switch_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Switch controller service not available, waiting...")
        # Client for C++ moveit node to execute pose goals 
        self.cpp_arm_client = self.create_client(MoveArm, "/move_arm",callback_group=r_callback_group)
        while not self.cpp_arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Move arm service not available, waiting...")
        # Service to start the global planner sequence
        self.start_service = self.create_service(StartGlobalSequence, "start_global_sequence", self.start_sequence_srv_callback, callback_group=r_callback_group)

        ### IMAGE PROCESSING
        self.image = None
        self.depth = None
        self.camera_info = None
        self.capture_flag = False
        # rate to wait for images to get saved
        self.rate = self.create_rate(1)
        # CV bridge to convert from image message
        self.br = CvBridge()

        ### TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


    def start_sequence_srv_callback(self, request, response):
        self.get_logger().info(f"Starting global planner sequence...")
        self.get_logger().info(f"Switching to joint_trajectory_controller...")
        # Switch to joint trajectory controller for normal moveit pose goal
        switch_result = self.switch_controller()
        if switch_result:
            self.get_logger().info(f"Succesfully switched to joint_trajectory_controller.")
        else:
            self.get_logger().error(f"Could not switch to joint_trajectory_controller. Aborting Global Planning Sequence")

        # Take one image from callback, wait until it is complete
        self.capture_flag = True
        try:
            while rclpy.ok() and self.capture_flag:
                self.get_logger().info("Taking capture of scene...")
                self.rate.sleep()
        except KeyboardInterrupt:
            pass

        # Use opencv to find closest apple pose
        apple_pose_optical = self.find_closest_apple()
        # Transform from optical to base_link frame on ur5
        pose_goal_base = self.transform_optical_to_base(apple_pose_optical)
        self.get_logger().info(f"Sending pose goal at: [{pose_goal_base.pose.position.x},{pose_goal_base.pose.position.y},{pose_goal_base.pose.position.z}] for execution...")
        # Send initial pose goal to C++ moveit service for initial movement
        result = self.send_pose_goal(pose_goal_base)

        # Check if pose goal was succesfully completed
        if result == False:
            self.get_logger().error(f"Failed to move to pose! Aborting Global Planning Sequence.")
            response = False
        else:
            self.get_logger().info(f"Successfully moved to pose goal in front of apple, ending global planner sequence...")
            response.result = True

        # return result, ending sequence
        return response

    def send_pose_goal(self, pose):
        # Sends x,y,z to C++ moveit node to execute pose goal since python moveit not available for humble
        request = MoveArm.Request()
        request.goal.x = pose.pose.position.x
        request.goal.y = pose.pose.position.y
        request.goal.z = pose.pose.position.z
        future = self.cpp_arm_client.call(request)
        return future

    def switch_controller(self):
        # Switches controller from forward position controller to joint_trajectory controller
        request = SwitchController.Request()
        request.activate_controllers = ["joint_trajectory_controller"]
        request.deactivate_controllers = ["forward_position_controller"]
        request.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()
        future = self.switch_controller_client.call(request)
        return future.ok

    def transform_optical_to_base(self, apple_position):
        # Creates a pose stamped message from apple position and transforms it from optical frame to base frame
        origin = PoseStamped()
        origin.header.frame_id = "camera_link_optical"
        origin.pose.position.x = float(apple_position[0])
        origin.pose.position.y = float(apple_position[1])
        origin.pose.position.z = float(apple_position[2])
        origin.pose.orientation.x = 0.0
        origin.pose.orientation.y = 0.0
        origin.pose.orientation.z = 0.0
        origin.pose.orientation.w = 1.0
        new_pose = self.tf_buffer.transform(origin, "base_link", rclpy.duration.Duration(seconds=1))
        return new_pose


    def find_closest_apple(self):
        # Finds the closest apple to the center of the image using the RGBD data
        if self.image is not None and self.depth is not None and self.camera_info is not None:
            image = self.image
            depth = self.depth
            # Convert to HSV and find red objects
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])
            # Threshold the HSV image to get only red colors
            mask = cv2.inRange(hsv, lower_red, upper_red)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # Get all apple x,y,z. returns the closest apple point to center of the camera
            apple_centers = []
            z_dist = []
            # create
            camera_model = PinholeCameraModel()
            camera_model.fromCameraInfo(self.camera_info)
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                # Get mask of depth and draw contours
                mask = np.zeros_like(depth)
                cv2.drawContours(mask, [contour], 0, (255), -1)
                # Calculate average distance of mask from depth image
                dist_to_apple = np.average(depth[np.nonzero(mask == 255)])
                z_dist.append(dist_to_apple)
                # Project the center of each apple to 3d then normalize it by the depth to get the 3d point
                xy_from_uv = camera_model.projectPixelTo3dRay((x+w//2, y+h//2))
                # Adjust z to put arm away from apple and not in apple
                apple_centers.append([xy_from_uv[0]*dist_to_apple, xy_from_uv[1]*dist_to_apple, dist_to_apple-.2])
            if apple_centers:
                min_idx = np.argmin(z_dist)
                return apple_centers[min_idx]            
        return None

    def rgbd_callback(self, rgb, depth, camera_info):
        # Saves a single frame after capture_flag trigger
        if self.capture_flag:
            # Convert ot opencv compatible image type
            self.image = self.br.imgmsg_to_cv2(rgb, "bgr8")
            self.depth = self.br.imgmsg_to_cv2(depth, "32FC1")
            self.camera_info = camera_info
            self.capture_flag = False

def main(args=None):
    rclpy.init(args=args)
    global_planner = GlobalPlanner()
    executor = MultiThreadedExecutor()
    rclpy.spin(global_planner, executor=executor)
    rclpy.shutdown()


if __name__ == "__main__":
    main()