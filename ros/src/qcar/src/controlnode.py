#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

class ControlNode(object):
    def __init__(self):
        super().__init__()
        rospy.loginfo("Starting Control Node")
        self.subscribers()
        self.publishers()
        self.now = rospy.Time.now()

    def subscribers(self):
        self.velocity_sub = rospy.Subscriber('/qcar/velocity', Float64, self.throttle_callback, queue_size=1, buff_size=2**24)
        self.steering_sub = rospy.Subscriber('/qcar/steering', Float64, self.steering_callback, queue_size=1, buff_size=2**24)

    def publishers(self):
        # Rear wheels for throttle
        self.rl_control_pub = rospy.Publisher('/qcar/rl_controller/command', Float64, queue_size=0)
        self.rr_control_pub = rospy.Publisher('/qcar/rr_controller/command', Float64, queue_size=0)

        # Front wheel hubs for steering
        self.fl_control_pub = rospy.Publisher('/qcar/base_fl_controller/command', Float64, queue_size=0)
        self.fr_control_pub = rospy.Publisher('/qcar/base_fr_controller/command', Float64, queue_size=0)

    def throttle_callback(self, value):
        self.rr_control_pub.publish(value)
        self.rl_control_pub.publish(Float64(-value.data))

    def steering_callback(self, value):
        self.fr_control_pub.publish(value)
        self.fl_control_pub.publish(value)

if __name__ == '__main__':
    rospy.init_node('control_node')
    r = ControlNode()
    rospy.spin()