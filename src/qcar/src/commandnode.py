#!/usr/bin/env python3

import threading
import inputs
import rospy
import time
from std_msgs.msg import Float64, Bool
from numpy import interp


class ControllerCommandNode(object):
    def __init__(self):
        super().__init__()

        gamepadThread = threading.Thread(target=self.monitorGamepad)
        gamepadThread.daemon = True
        gamepadThread.start()

        self.commands = {
            'throttle': 0,
            'steering': 0,
            'autonomous': False
        }

        self.throttle_pub = rospy.Publisher('/qcar/velocity_target', Float64, queue_size=100)
        self.steering_pub = rospy.Publisher('/qcar/steering_target', Float64, queue_size=100)
        self.autonomous_pub = rospy.Publisher('/qcar/autonomous_enabled', Bool, queue_size=100)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_command(self.commands['throttle'], self.commands['steering'], self.commands['autonomous'])
            rate.sleep()

    def monitorGamepad(self):
        while True:
            try:
                for event in inputs.get_gamepad():
                    if (event.code == 'ABS_RX'):
                        self.commands['steering'] = event.state
                    if (event.code == 'ABS_RZ'):
                        self.commands['throttle'] = event.state
                    if (event.code == 'BTN_TL'):
                        self.commands['autonomous'] = not self.commands['autonomous']

            except inputs.UnpluggedError:
                time.sleep(0.5)

    def publish_command(self, throttle, steering, autonomous):
        throttle_remapped = interp(throttle, [0, 255], [0, 30])
        steering_adjusted = steering - 128
        steering_remapped = interp(steering_adjusted, [-32256, 32256], [1, -1])
        if not self.commands['autonomous']:
            self.throttle_pub.publish(Float64(throttle_remapped))
            self.steering_pub.publish(Float64(steering_remapped))
        self.autonomous_pub.publish(Bool(autonomous))

if __name__ == '__main__':
    rospy.init_node('command_node')
    r = ControllerCommandNode()
    rospy.spin()

