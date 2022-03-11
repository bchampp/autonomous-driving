#!/usr/bin/env python3

from __future__ import division, print_function, absolute_import

import rospy
import numpy as np
from qcar.product_QCar import QCar
from qcar.q_interpretation import *

from std_msgs.msg import String, Float64
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import BatteryState
import time

class QCarNode(object):

	def __init__(self):
		super().__init__()
		self.battery_pub_ = rospy.Publisher('/qcar/battery_state', BatteryState, queue_size=10)
		self.carvel_pub_ = rospy.Publisher('/qcar/velocity_actual', Vector3Stamped, queue_size=10)

		self.myCar = QCar()
		self.sample_time = 0.001
		self.throttle = 0.0
		self.steering = 0.0
		self.velocity_sub = rospy.Subscriber('/qcar/velocity_target', Float64, self.process_velocity, queue_size=10)
		self.steering_sub = rospy.Subscriber('/qcar/steering_target', Float64, self.process_steering, queue_size=10)

	def looping(self):
		rate = rospy.Rate(50)

		while not rospy.is_shutdown():
			# Generate Commands
			LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])
			self.command = np.array([self.throttle, self.steering])
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
			rate.sleep()

		self.myCar.terminate()
	
	def process_velocity(self, speed):
		self.throttle = speed.data / 30.0

	def process_steering(self, steering):
		self.steering = steering.data

if __name__ == '__main__':
	rospy.init_node('qcar_node')
	r = QCarNode()
	r.looping()
	rospy.spin()
	# print('Im here')