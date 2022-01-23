#!/usr/bin/env python3

from __future__ import division, print_function, absolute_import

import roslib
import rospy
import numpy as np
from qcar.product_QCar import QCar
from qcar.q_interpretation import *

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import BatteryState
import time

class QcarNode(object):

	def __init__(self):
		super().__init__()
		self.battery_pub_ = rospy.Publisher('/qcar/battery_state', BatteryState, queue_size=10)
		self.carvel_pub_ = rospy.Publisher('/qcar/velocity', Vector3Stamped, queue_size=10)

		self.myCar = QCar()
		self.sample_time = 0.001
		self.command = np.array([0, 0])
		self.cmd_sub_ = rospy.Subscriber('/qcar/user_command', Vector3Stamped, self.process_cmd, queue_size=100)
#-------------------------------------------------------------------------------------------------
	def looping(self):
		
		while not rospy.is_shutdown():
			# Generate Commands
			LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])
			# print(self.command)
			# talk to QCar
			current, batteryVoltage, encoderCounts = self.myCar.read_write_std(self.command, LEDs)     
			

			battery_state = BatteryState()
			battery_state.header.stamp = rospy.Time.now() 
			battery_state.header.frame_id = 'battery_voltage'
			battery_state.voltage = batteryVoltage
			self.battery_pub_.publish(battery_state)

			longitudinal_car_speed = basic_speed_estimation(encoderCounts)
			velocity_state = Vector3Stamped()
			velocity_state.header.stamp = rospy.Time.now() 
			velocity_state.header.frame_id = 'car_velocity'
			velocity_state.vector.x = float(np.cos(self.command[1]) * longitudinal_car_speed)
			velocity_state.vector.y = float(np.sin(self.command[1]) * longitudinal_car_speed)
			self.carvel_pub_.publish(velocity_state)
		
			time.sleep(self.sample_time)
		

		self.myCar.terminate()
	
	
	def process_cmd(self, sub_cmd):
		vel_cmd = sub_cmd.vector.x
		str_cmd = sub_cmd.vector.y - 0.01
		self.command = np.array([vel_cmd, str_cmd])
		


if __name__ == '__main__':
	rospy.init_node('qcar_node')
	r = QcarNode()
	r.looping()
	rospy.spin()
	# print('Im here')