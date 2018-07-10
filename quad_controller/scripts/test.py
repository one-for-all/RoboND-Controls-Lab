#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose


def pose_msg_callback(pose_msg):
    print(pose_msg)


if __name__ == "__main__":
    rospy.init_node("test_node")

    pose_sub = rospy.Subscriber("/quad_rotor/pose", Pose, pose_msg_callback, queue_size=1)

    while not rospy.is_shutdown():
        rospy.spin()
