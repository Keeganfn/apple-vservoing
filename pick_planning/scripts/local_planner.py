#!/usr/bin/env python3

# ROS
import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
# Interfaces
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, TwistStamped
from pick_planning_interfaces.srv import MoveArm, StartSequence
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


class LocalPlanner(Node):

    def __init__(self):
        super().__init__('local_planner_node')
        ### Subscribers/ Publishers
        self.subscription = Subscriber(self,Image,'/camera2/image_raw')
        self.depth_sub = Subscriber(self,Image,'/camera2/depth/image_raw')
        self.ts = ApproximateTimeSynchronizer([self.subscription, self.depth_sub],30,0.05,)
        self.ts.registerCallback(self.rgbd_servoing_callback)
        # Publisher to end effector servo controller, sends velocity commands
        self.servo_publisher = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        #specify reentrant callback group 
        r_callback_group = ReentrantCallbackGroup()

        ### Services
        # Service to start the local planner sequence
        self.start_service = self.create_service(StartSequence, "start_local_sequence", self.start_sequence_srv_callback, callback_group=r_callback_group)

        ### Servo controller params
        self.max_vel = 0.8
        self.smoothing_factor = 12
        self.z_speed = 0.5
        self.start_flag = False
        # rate to wait for images to get saved
        self.rate = self.create_rate(1)

        ### Image Processing
        self.br = CvBridge()

        ### Tf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def start_sequence_srv_callback(self, request, response):
        # Starts servo node it it hasnt been started already
        self.get_logger().info("Activating servo node...")
        self.get_logger().info("Switching controller to forward_position_controller...")
        self.start_flag = True
        self.get_logger().info("Starting visual arm servoing...")
        # Servos until we are in front of apple
        try:
            while rclpy.ok() and self.start_flag:
                self.get_logger().info("Servoing arm in front of apple...")
                self.rate.sleep()
        except KeyboardInterrupt:
            pass
        self.get_logger().info("Successfully servoed in front of the apple!")
        response.result=True
        return response

    def normalize(self, val, minimum, maximum):
        # Normalizes val between min and max
        return (val - minimum) / (maximum-minimum)

    def transform_optical_to_ee(self, x, y):
        # Transforms from optical frame to end effector frame
        origin = PoseStamped()
        origin.header.frame_id = "camera_link_optical"
        origin.pose.position.x = x
        origin.pose.position.y = y
        origin.pose.position.z = 0.0
        origin.pose.orientation.x = 0.0
        origin.pose.orientation.y = 0.0
        origin.pose.orientation.z = 0.0
        origin.pose.orientation.w = 1.0
        new_pose = self.tf_buffer.transform(origin, "tool0", rclpy.duration.Duration(seconds=1))
        return new_pose

    def exponential_vel(self, vel):
        # Exponential function for determining velocity scaling based on pixel distance from center of the camera 
        # Bounded by 0 and max_vel
        return (1-np.exp((-self.smoothing_factor/2)*vel)) * self.max_vel

    def create_servo_vector(self, closest_apple, image, distance):
        # Centerpoints
        apple_x = closest_apple[0]
        apple_y = closest_apple[1]
        center_x = image.shape[1] // 2
        center_y = image.shape[0] // 2
        # Get x magnitude and set velocity with exponential function * max_vel
        if apple_x >= center_x:
            new_x = self.exponential_vel(self.normalize(apple_x, center_x, image.shape[1]))
        else:
            new_x = -self.exponential_vel(1-self.normalize(apple_x, 0, center_x))
        # Get y magnitude
        if apple_y >= center_y:
            new_y = self.exponential_vel(self.normalize(apple_y, center_y, image.shape[0]))
        else:
            new_y = -self.exponential_vel(1 - self.normalize(apple_y, 0, center_y))
        
        # Transform from optical frame to end effector frame
        transformed_vector = self.transform_optical_to_ee(new_x, new_y)

        # Create Twiststamped message in end effector frame
        vel_vec = TwistStamped()
        vel_vec.header.stamp = self.get_clock().now().to_msg()
        vel_vec.header.frame_id = "tool0"
        vel_vec.twist.linear.x = transformed_vector.pose.position.x
        vel_vec.twist.linear.y = transformed_vector.pose.position.y

        # If we are within picking distance then stop, otherwise move forward
        if distance < .05:
            vel_vec.twist.linear.z = 0.0
            vel_vec.twist.linear.x = 0.0
            vel_vec.twist.linear.y = 0.0
            self.start_flag = False
        else:
            vel_vec.twist.linear.z = self.z_speed
        return vel_vec


    def rgbd_servoing_callback(self, rgb, depth):
        if self.start_flag:
            # Convert to opencv format from msg
            image = self.br.imgmsg_to_cv2(rgb, "bgr8")
            depth = self.br.imgmsg_to_cv2(depth, "32FC1")
            
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            # Define the range of red color in HSV
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])
            # Threshold the HSV image to get only red colors
            mask = cv2.inRange(hsv, lower_red, upper_red)
            # Find contours that match criteria
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            apple_centers = []
            z_dist = []
            for contour in contours:
                # Get center of apple contour
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                apple_centers.append([x+w//2, y+h//2])
                # Get average depth of apple contour using depth image
                mask = np.zeros_like(depth)
                cv2.drawContours(mask, [contour], 0, (255), -1)
                dist_to_apple = np.average(depth[np.nonzero(mask == 255)])
                z_dist.append(dist_to_apple)

            if apple_centers:
                # Get closest apple to center of camera (z distance), create a servo vector, then publish
                closest_apple = apple_centers[np.argmin(z_dist)]
                # cv2.circle(image, (closest_apple[0], closest_apple[1]), 5, (0, 255, 0), 2)
                try:
                    vec = self.create_servo_vector(closest_apple, image, dist_to_apple)
                    self.servo_publisher.publish(vec)
                except TransformException as e:
                    self.get_logger().info(f'Transform failed: {e}')



def main(args=None):
    rclpy.init(args=args)
    local_planner = LocalPlanner()
    executor = MultiThreadedExecutor()
    rclpy.spin(local_planner, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()