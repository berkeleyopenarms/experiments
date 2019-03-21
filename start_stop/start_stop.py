#!/usr/bin/env python3

# A basic example of using BlueInterface for joint positions control.
# It allows a user to record four sets of joint positions by manually moving the arm to each
# position and pressing enter. It then plays back a trajectory comprised of the four sets of
# joint positions in an infinite loop.

import sys
from blue_interface import BlueInterface
import numpy as np
import time

if __name__ == '__main__':

    blue_right = BlueInterface("right", "127.0.0.1")
    blue_left = BlueInterface("left", "127.0.0.1")

    while True:

        blue_right.disable_control()
        blue_left.disable_control()

        blue_right.disable_gripper()
        blue_left.disable_gripper()
        input("Press [Enter] to hold position...")

        blue_right.set_joint_positions(blue_right.get_joint_positions())
        blue_left.set_joint_positions(blue_left.get_joint_positions())

        blue_right.command_gripper(blue_right.get_gripper_position(), effort=5.0, wait=False)
        blue_left.command_gripper(blue_left.get_gripper_position(), effort=5.0, wait=False)
        input("Press [Enter] to for gravity compensation mode...")

