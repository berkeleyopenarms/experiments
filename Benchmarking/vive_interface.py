#!/usr/bin/env python

import rospy
import sys
import actionlib
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped

from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal,
)

def callback(data):
    rospy.loginfo("x: ", data.Pose.Point.x)

def listener():
    rospy.init_node('vive_interface', anonymouse=True)

    rospy.Subscriber("controller_pose", PoseStamped, callback, queue_size = 1)

    rospy.spin()

if __name__ == '__main__':
    listener()