#!/usr/bin/env python

import rospy
import gimbal_data
from math import pi
from std_msgs.msg import Float32, Float64
from vesc_msgs.msg import VescStateStamped

# gimbal_data.lookup_table

def position_callback(data):
   index = int((data.data / (2.0 * pi)) * len(gimbal_data.lookup_table))
   index %= len(gimbal_data.lookup_table)
   current_pub.publish(gimbal_data.lookup_table[index] / 2)
   print gimbal_data.lookup_table[index]

rospy.init_node('measure')

current_pub = rospy.Publisher('/commands/motor/current', Float64, queue_size=1)
position_pub = rospy.Subscriber('/sensors/rotor_position', Float32, position_callback)

rospy.spin()
