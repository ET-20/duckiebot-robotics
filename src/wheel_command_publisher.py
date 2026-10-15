#!/usr/bin/python3

import rospy
import os
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Header



class WheelCommandPublisher:

    def __init__(self, robot_name): 
        # initialize a node with a name, annonymous=True ensures that the name is unique
        rospy.init_node('wheel_command_publisher', anonymous=True)

        # create a publisher for a topic of type duckietown_msgs.msg.WheelsCmdStamped
        topic = '/' + robot_name + '/wheels_driver_node/wheels_cmd'
        self.publisher = rospy.Publisher(topic, WheelsCmdStamped, queue_size = 10)
        rospy.sleep(2.0)
        
        # create a WheelsCmdStamped message
        self.command = WheelsCmdStamped()
        self.command.header = Header()
   

    def turn_wheels(self, velocity):
        self.command.vel_left = velocity
        self.command.vel_right = velocity

        self.command.header.stamp = rospy.Time.now()
        self.publisher.publish(self.command)
        rospy.loginfo("Publishing wheel command")

    def run(self):
        for i in range(5):
            velocity = i / 10
            self.turn_wheels(velocity)
            print(velocity)
            rospy.sleep(1) # wait 1 seconds

        for i in range(4, -1, -1):
            velocity = i / 10
            self.turn_wheels(velocity)
            print(velocity)
            rospy.sleep(1)

        # stop wheels
        self.turn_wheels(0.0)

if __name__ == '__main__':
    vehicle_name = os.getenv('VEHICLE_NAME', 'alpha')
    wcp = WheelCommandPublisher(vehicle_name)
    wcp.run()
