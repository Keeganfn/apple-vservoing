#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from geometry_msgs.msg import TwistStamped, PoseStamped, Vector3, Pose
import tf2_geometry_msgs
import math
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        # self.subscription = self.create_subscription(Image,'/camera2/image_raw',self.listener_callback,10)
        self.subscription = Subscriber(self,Image,'/camera2/image_raw')
        self.depth_sub = Subscriber(self,Image,'/camera2/depth/image_raw')
        self.servo_publisher = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.cli = self.create_client(Trigger, "/servo_node/start_servo")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        activated = self.activate_servo_node()

        self.switch_controller_client = self.create_client(SwitchController, "/controller_manager/switch_controller")
        while not self.switch_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        activated2 = self.switch_controller()
        print("LSDKFJSDLKFJSDFLKJ2", activated2)

        self.max_vel = 0.8
        self.smoothing_factor = 12
        self.br = CvBridge()
        queue_size = 30
        self.ts = ApproximateTimeSynchronizer([self.subscription, self.depth_sub],queue_size,0.01,)
        self.ts.registerCallback(self.listener_callback)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


    def switch_controller(self):
        self.request = SwitchController.Request()
        self.request.activate_controllers = ["forward_position_controller"]
        self.request.deactivate_controllers = ["joint_trajectory_controller"]
        self.future = self.switch_controller_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def activate_servo_node(self):
        self.request = Trigger.Request()
        self.future = self.cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def normalize(self, val, minimum, maximum):
        return (val - minimum) / (maximum-minimum)

    def transform_optical_to_ee(self, x, y):
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
        return (1-np.exp((-self.smoothing_factor/2)*vel)) * self.max_vel

    def create_servo_vector(self, closest_apple, image, distance):
        apple_x = closest_apple[0]
        apple_y = closest_apple[1]
        center_x = image.shape[1] // 2
        center_y = image.shape[0] // 2
        # get x magnitude
        if apple_x >= center_x:
            new_x = self.exponential_vel(self.normalize(apple_x, center_x, image.shape[1]))
        else:
            new_x = -self.exponential_vel(1-self.normalize(apple_x, 0, center_x))
        # get y magnitude
        if apple_y >= center_y:
            new_y = self.exponential_vel(self.normalize(apple_y, center_y, image.shape[0]))
        else:
            new_y = -self.exponential_vel(1 - self.normalize(apple_y, 0, center_y))
        
        transformed_vector = self.transform_optical_to_ee(new_x, new_y)

        vel_vec = TwistStamped()
        vel_vec.header.stamp = self.get_clock().now().to_msg()
        vel_vec.header.frame_id = "tool0"
        vel_vec.twist.linear.x = transformed_vector.pose.position.x
        vel_vec.twist.linear.y = transformed_vector.pose.position.y
        if distance < .05:
            vel_vec.twist.linear.z = 0.0
        else:
            vel_vec.twist.linear.z = 0.3

        return vel_vec


    def listener_callback(self, msg, msg2):
        depth = self.br.imgmsg_to_cv2(msg2, "32FC1")
        # depth = cv2.normalize(depth, depth, 0,1,cv2.NORM_MINMAX)

        image = self.br.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Define the range of red color in HSV
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv, lower_red, upper_red)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        apple_centers = []
        xy_dist = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            apple_centers.append([x+w//2, y+h//2])
            xy_dist.append(math.sqrt((image.shape[1]//2 - x+w//2)**2 + (image.shape[0]//2 - y+h//2)**2))

        if apple_centers:
            closest_contour = contours[np.argmin(xy_dist)]
            mask = np.zeros_like(depth)
            cv2.drawContours(mask, [closest_contour], 0, (255), -1)
            dist_to_apple = np.average(depth[np.nonzero(mask == 255)])
            closest_apple = apple_centers[np.argmin(xy_dist)]
            cv2.circle(image, (closest_apple[0], closest_apple[1]), 5, (0, 255, 0), 2)
            print("DISTANCE", dist_to_apple)
            print(np.argmin(xy_dist))

            try:
                vec = self.create_servo_vector(closest_apple, image, dist_to_apple)
                self.servo_publisher.publish(vec)
            except TransformException as e:
                self.get_logger().info(f'Transform failed: {e}')


        cv2.imshow("camera", image)
        cv2.waitKey(1)

        cv2.imshow("mask", depth)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    minimal = MinimalSubscriber()
    rclpy.spin(minimal)
    print("Made it!")
    rclpy.shutdown()


if __name__ == '__main__':
    main()