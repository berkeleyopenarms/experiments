#!/usr/bin/env python

import csv
import rospy
from math import pi
from std_msgs.msg import Float64, Float32
from std_srvs.srv import Trigger
from vesc_msgs.msg import VescStateStamped
import sys

class Everything:
    unwinding = False
    fullywound_offset = 0

    force_zero = None
    tacho_zero = None
    tacho_prev = None
    current = 0
    force = 0
    force_avg = 0

    not_moving_counter = 0

    data_current = {}
    data_tacho = {}

    def __init__(self):
        rospy.init_node('measure')

        if len(sys.argv) != 2 or float(sys.argv[1]) < 0 or float(sys.argv[1]) > 0.1:
            rospy.logerr("bad argument for stepper position")
            rospy.signal_shutdown("bad argument for stepper position")
            exit()
            return 0

        self.duty_pub = rospy.Publisher("commands/motor/duty_cycle", Float64, queue_size=2)
        self.current_pub = rospy.Publisher("commands/motor/current", Float64, queue_size=2)

        float64_msg = Float64()
        float64_msg.data = 0
        self.duty_pub.publish(float64_msg)

        rospy.loginfo("Zeroing stepper...")
        rospy.wait_for_service('zero_motor')
        zero_motor = rospy.ServiceProxy('zero_motor', Trigger)
        rospy.loginfo(zero_motor())

        self.stepper_pub = rospy.Publisher("stepper_position", Float32, queue_size=2)
        position_msg = Float32()
        position_msg.data = float(sys.argv[1])
        for i in range(100):
            self.stepper_pub.publish(position_msg)
            rospy.sleep(0.1)

        rospy.Subscriber("/strain_gauge", Float32, self.force_cb)

        rospy.Subscriber("/sensors/core", VescStateStamped, self.vesc_cb)

        rospy.loginfo("Publishing motor command")
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            # rospy.spinOnce()
            r.sleep()

    def force_cb(self, data):
        if self.force_zero == None:
            self.force_zero = data.data
        self.force = data.data

    def vesc_cb(self, data):
        tacho = data.state.displacement

        if self.unwinding:
            duty_msg = Float64()
            duty_msg.data = -0.15
            self.duty_pub.publish(duty_msg)

            offset = tacho - self.tacho_zero
            if (self.fullywound_offset > 0 and offset < 0) or (self.fullywound_offset < 0 and offset > 0):
                duty_msg.data = 0
                self.duty_pub.publish(duty_msg)
                rospy.loginfo("finished probably")
                rospy.signal_shutdown("dieded")
                exit()

            return

        if self.tacho_zero == None:
            self.tacho_zero = tacho

        if self.tacho_prev != None and abs(self.tacho_prev - tacho) < 35:
            self.force_avg += self.force
            self.not_moving_counter += 1
        else:
            self.not_moving_counter = 0
            self.force_avg = 0
            self.tacho_prev = tacho

        if data.header.seq % 2 == 0:
            self.data_tacho[tacho - self.tacho_zero] = self.force

        if self.not_moving_counter > 4 * 50:
            self.force_avg /= self.not_moving_counter

            self.data_current[self.current] = self.force_avg - self.force_zero
            rospy.loginfo("not moving so something happened")
            rospy.loginfo(self.data_current)
            rospy.loginfo(self.data_tacho)

            if self.current >= 0.999:
                self.write_csv("data_" + sys.argv[1] + "_current.csv", self.data_current)
                self.write_csv("data_" + sys.argv[1] + "_tacho.csv", self.data_tacho)

                self.unwinding = True
                self.fullywound_offset = (tacho - self.tacho_zero)

            self.current += 0.05
            self.not_moving_counter = 0

        current_msg = Float64()
        current_msg.data = self.current
        self.current_pub.publish(current_msg)

    def write_csv(self, filename, dictionary):
        with open(filename,'wb') as f:
            w = csv.writer(f)
            w.writerows(dictionary.items())

if __name__ == '__main__':
    Everything()
