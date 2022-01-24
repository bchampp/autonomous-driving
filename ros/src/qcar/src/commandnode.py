#!/usr/bin/env python3

from __future__ import division, print_function, absolute_import

import roslib
import rospy
import numpy as np
from qcar.q_misc import *
from qcar.q_ui import *
from qcar.q_interpretation import *

from geometry_msgs.msg import Vector3Stamped
import time

class CommandNode(object):
	def __init__(self):
		super().__init__()
		self.gpad = gamepadViaTarget(1)
		self.cmd_pub_ = rospy.Publisher('/qcar/user_command', Vector3Stamped, queue_size=100)
		while not rospy.is_shutdown():
			# left_lateral, left_longitudinal, right_lateral, right_longitudinal, LT, RT, A, B, X, Y, LB, RB, BACK, START, Logitech, hat = gamepad_io_qcar() # .................... Logitech......................
			new = self.gpad.read()
			pose = control_from_gamepad(self.gpad.LB, self.gpad.RT, self.gpad.LLA, self.gpad.A)
			self.process_command(pose, new)
			time.sleep(0.01)
		self.gpad.terminate()
#--------------------------------------------------------------------------------------------

	def process_command(self, pose, new):
		if new:
			pub_cmd = Vector3Stamped()
			pub_cmd.header.stamp = rospy.Time.now() 
			pub_cmd.header.frame_id = 'command_input'
			pub_cmd.vector.x = float(pose[0])
			pub_cmd.vector.y = float(pose[1])
			self.cmd_pub_.publish(pub_cmd)

if __name__ == '__main__':
	rospy.init_node('command_node')
	r = CommandNode()

	rospy.spin()
	