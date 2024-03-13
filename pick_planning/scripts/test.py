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

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(Image,'/camera2/image_raw',self.listener_callback,10)
        self.servo_publisher = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.max_vel = 0.8
        self.br = CvBridge()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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


    def create_servo_vector(self, closest_apple, image):
        apple_x = closest_apple[0]
        apple_y = closest_apple[1]
        center_x = image.shape[1] // 2
        center_y = image.shape[0] // 2
        # get x magnitude
        if apple_x >= center_x:
            new_x = self.normalize(apple_x, center_x, image.shape[1])
        else:
            new_x = self.normalize(apple_x, 0, center_x)
        # get y magnitude
        if apple_y >= center_y:
            new_y = self.normalize(apple_y, center_y, image.shape[0])
        else:
            new_y = self.normalize(apple_y, 0, center_y)
        
        new_x *= self.max_vel 
        new_y *= self.max_vel 

        print(new_x, new_y)
        transformed_vector = self.transform_optical_to_ee(new_x, new_y)

        vel_vec = TwistStamped()
        vel_vec.header.stamp = self.get_clock().now().to_msg()
        vel_vec.header.frame_id = "tool0"
        vel_vec.twist.linear.x = transformed_vector.pose.position.x
        vel_vec.twist.linear.y = transformed_vector.pose.position.y
        vel_vec.twist.linear.z = 0.0

        return vel_vec


    def listener_callback(self, msg):
        print("In callback")

        image = self.br.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Define the range of red color in HSV
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv, lower_red, upper_red)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        apple_centers = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            apple_centers.append([x+w//2, y+h//2])

        if apple_centers:
            closest_apple = min(apple_centers)

            try:
                vec = self.create_servo_vector(closest_apple, image)
                self.servo_publisher.publish(vec)
            except TransformException as e:
                self.get_logger().info(f'Transform failed: {e}')


        # cv2.imshow("camera", image)
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    minimal = MinimalSubscriber()
    rclpy.spin(minimal)
    print("Made it!")
    rclpy.shutdown()


if __name__ == '__main__':
    main()