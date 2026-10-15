#!/usr/bin/env python3

import rospy
import os
import math
import tf2_ros
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

def main():
    rospy.init_node('virtual_robot_tf_publisher')
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(20)  # 20 Hz
    vehicle_name = os.getenv('VEHICLE_NAME', 'alpha')


    parent_frame = "map"  # or "map", as appropriate
    child_frame = vehicle_name+"/base"
    t = 0.0
    r = 0.01

    while not rospy.is_shutdown():
        x = math.cos(t) * r
        y = math.sin(t) * r
        z = 0.0
        yaw = t  + math.pi/2 # robot rotates as it moves

        quat = quaternion_from_euler(0, 0, yaw)

        tf_msg = geometry_msgs.msg.TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = parent_frame
        tf_msg.child_frame_id = child_frame
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = z
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]

        br.sendTransform(tf_msg)

        t += 0.02
        r += 0.001
        rate.sleep()

if __name__ == '__main__':
    main()