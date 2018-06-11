import sys
sys.path.append('../../blue_interface')
from blue_interface import BlueInterface #this is the API for the robot

kk = BlueInterface("left","hekate.cs.berkeley.edu") #creates object of class KokoInterface at the IP in quotes with the name 'kk'
kk.disable_control() #this turns off any other control currently on the robot (leaves it in gravtiy comp mode)

p1 = kk.get_joint_positions()

raw_input('Press enter to go to the position')

kk.set_joint_positions(p1)