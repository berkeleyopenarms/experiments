#!/usr/bin/env python

import rospy
from math import pi
from std_msgs.msg import Float32, Float64
from vesc_msgs.msg import VescStateStamped

class measure(object):
   def __init__(self):
      self.state_sub = rospy.Subscriber('/sensors/core', VescStateStamped, self.state_callback)
      self.position_sub = rospy.Subscriber('/sensors/rotor_position', Float32, self.rotor_position_callback)

      self.current = 0
      self.position = 0

      self.new_current = False
      self.new_position = False

   def state_callback(self, data):
      self.current = data.state.current_motor
      self.new_current = True

   def rotor_position_callback(self, data):
      self.position = data.data
      self.new_position = True

   def get_data(self):
      if not self.new_position or not self.new_current:
         return None
      self.new_current = False
      self.new_position = False
      return (self.current, self.position)

if __name__ == '__main__':
    rospy.init_node('measure')
    x = measure()
    r = rospy.Rate(50)

    datapoints = []
    while not rospy.is_shutdown():
       data = x.get_data()
       if data != None:
          datapoints.append(data)
          if len(datapoints) % 100 == 0:
             print len(datapoints)
          if len(datapoints) % 1000 == 0:
             print datapoints
             print "\n\n\n"
       r.sleep()
       # rospy.spinOnce()
