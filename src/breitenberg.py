#!/usr/bin/python3

import rospy
import os
import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Header


class Breitenberg:

    def __init__(self, robot_name):
        rospy.init_node('breitenberg', anonymous=True)

        self.wheel_command_publisher = rospy.Publisher(
            '/' + robot_name + '/wheels_driver_node/wheels_cmd',
            WheelsCmdStamped,
            queue_size=1
        )

        self.wheel_command = WheelsCmdStamped()
        self.wheel_command.header = Header()
        self.camera_image_msg = CompressedImage()
        self.cv2_bridge = CvBridge()

        rospy.Subscriber(
            '/' + robot_name + '/camera_node/image/compressed',
            CompressedImage,
            self.callback,
            queue_size=1,
            buff_size=2**30
        )

        self.rate = rospy.Rate(20)
        rospy.sleep(2.0)
        rospy.on_shutdown(self.stop_wheels)


    def run(self):
        last_image_stamp = self.camera_image_msg.header.stamp

        while not rospy.is_shutdown():
            if not self.camera_image_msg.header.stamp == last_image_stamp:

                vel_left, vel_right = self.do_image_analysis()
                self.turn_wheels(vel_left, vel_right)

                last_image_stamp = self.camera_image_msg.header.stamp

            self.rate.sleep()


    def callback(self, data):
        self.camera_image_msg = data


    def do_image_analysis(self):

        # convert compressed image to cv2 image array
        image = self.cv2_bridge.compressed_imgmsg_to_cv2(
            self.camera_image_msg,
            "bgr8"
        )

        # convert image to HSV
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # yellow HSV range
        yellow_lower_hsv = np.array([20, 100, 100])
        yellow_upper_hsv = np.array([35, 255, 255])

        # create yellow mask
        mask = cv2.inRange(
            image_hsv,
            yellow_lower_hsv,
            yellow_upper_hsv
        )

        # divide image into left and right half
        height, width = mask.shape
        middle = width // 2

        left_half = mask[:, :middle]
        right_half = mask[:, middle:]

        # sum yellow pixels
        left_sum = np.sum(left_half)
        right_sum = np.sum(right_half)

        print("Left sum:", left_sum)
        print("Right sum:", right_sum)

        # calculate percentage of yellow pixels
        left_ratio = left_sum / (left_half.size * 255)
        right_ratio = right_sum / (right_half.size * 255)

        print("Left ratio:", left_ratio)
        print("Right ratio:", right_ratio)

        # calculate wheel velocities
        default_speed = 0.1
        gain = 2.0

        vel_left = default_speed + right_ratio * gain
        vel_right = default_speed + left_ratio * gain

        return vel_left, vel_right


    def turn_wheels(self, vel_left, vel_right):

        self.wheel_command.vel_left = vel_left
        self.wheel_command.vel_right = vel_right

        # set timestamp
        self.wheel_command.header.stamp = rospy.Time.now()

        # publish wheel command
        self.wheel_command_publisher.publish(self.wheel_command)

        rospy.loginfo(
            "left_command=" + str(self.wheel_command.vel_left)
            + " right_command=" + str(self.wheel_command.vel_right)
        )


    def stop_wheels(self):
        self.turn_wheels(0.0, 0.0)


if __name__ == '__main__':
    vehicle_name = os.getenv('VEHICLE_NAME', 'alpha')
    b = Breitenberg(vehicle_name)
    b.run()