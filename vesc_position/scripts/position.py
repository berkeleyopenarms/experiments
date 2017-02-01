import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from pid import PID

class vesc_position(object):
    def __init__(self):
        self.position_pid = PID(1, 0, 0)
        self.position_pid.setpoint = 0

        self.setpoint = 0
        self.command_pub = rospy.Publisher("/commands/motor/current",Int32, queue_size=2)
        self.encoder_sub = rospy.Subscriber("/encoder_angle", Int32, self.update)

    def update(self, angle):
        self.command_pub.publish(self.position_pid.calc(angle.data))

if __name__ == '__main__':
    rospy.init_node('vesc_position')
    vesc_position()
    rospy.spin()
