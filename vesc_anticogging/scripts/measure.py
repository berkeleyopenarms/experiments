#!/usr/bin/env python

import rospy
from math import pi
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped

class measure(object):
   def __init__(self):
      self.position_pub = rospy.Publisher('/commands/motor/position', Float64, queue_size=1)
      self.state_sub = rospy.Subscriber('/sensors/core', VescStateStamped, self.state_callback)
      r = rospy.Rate(20)

      self.handler = measurement_handler()

      self.steps = 360 * 4 # 0.25deg current measurement
      self.angle_step = 2 * pi / self.steps
      self.count = 0
      self.current_angle = 0

      self.position_pub.publish(self.current_angle)
      rospy.sleep(1)

   def state_callback(self, data):
      flag = self.handler.push_measurement(data.state.current_motor)
      if flag:
         self.count += 1

         if self.count >= self.steps:
            exit()

         self.current_angle = self.angle_step * self.count
         rospy.sleep(0.25)
         self.position_pub.publish(self.current_angle)
         rospy.loginfo("New position: " + str(self.current_angle))
         rospy.loginfo(self.handler.output)


   def pub(self):
      self.position_pub.publish(self.current_angle)

class measurement_handler(object):
   def __init__(self):
      self.output = []
      self.buffer = []
      self.current_angle = 0

   def push_measurement(self, current):
      self.buffer.append(current)

      if len(self.buffer) > 200:
         self.buffer.sort()
         self.buffer = self.buffer[50:150]

         avg = sum(self.buffer) * 1.0 / len(self.buffer)
         self.output.append(avg)

         self.buffer = []

         return True

      return False

if __name__ == '__main__':
    rospy.init_node('measure')
    rospy.loginfo("Publishing motor command")
    x = measure()
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
       x.pub()
       r.sleep()
       # rospy.spinOnce()
