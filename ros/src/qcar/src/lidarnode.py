#!/usr/bin/env python3

from __future__ import division, print_function, absolute_import

import roslib
import rospy
import numpy as np
import cv2
from qcar.q_essential import LIDAR

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import time

class LIDARNode(object):
	def __init__(self):
		super().__init__()
		self.lidar_pub = rospy.Publisher('/scan', LaserScan, queue_size=1000)
		self.num_measurements = 720
		self.lidar = LIDAR(num_measurements=self.num_measurements)
		self.distances = np.zeros((self.num_measurements,1))
		self.angles = np.zeros((self.num_measurements,1))
		self.sampleTime = 1 / 30
		#Get readings
		while not rospy.is_shutdown():
			# sampleTime = 1 / 30
			starTime = time.time()
			self.lidar.read()
			mid_time = time.time()
			scan_time = mid_time - starTime
			# print(scan_time)
			# Calculate the computation time, and the time that the thread should pause/sleep for
			computationTime = scan_time
			sleepTime = self.sampleTime - ( computationTime % self.sampleTime )
			
			# Pause/sleep and print out the current timestamp
			time.sleep(sleepTime)
			# counter += 1
			self.process_lidar_data(self.lidar_pub, self.lidar.distances.astype(np.float32), self.num_measurements, scan_time)
			process_time = time.time() - mid_time
			# print(process_time, scan_time)
		self.lidar.terminate()
			
#--------------------------------------------------------------------------------------------------------------
	def process_lidar_data(self, pub_lidar, distances, num_measurement, scan_times):

		scan = LaserScan()
		scan.header.stamp = rospy.Time.now()
		scan.header.frame_id = 'lidar'
		scan.angle_min = 0.0
		scan.angle_max = 6.2744
		# scan.angle_increment = 6.2744 / num_measurement
		# scan.time_increment = scan_times / num_measurement
		scan.angle_increment = 0.0174532923847
		scan.time_increment = 0.000132342218421
		scan.scan_time = scan_times
		scan.range_min = 0.15
		scan.range_max = 12.0
		# print('before')
		scan.ranges = distances.tolist()
		# scan.ranges = []
		# print('after')
		# for i in range(num_measurement):
		# 	scan.ranges.append(distances[i])
		pub_lidar.publish(scan)


if __name__ == '__main__':
	rospy.init_node('lidar_node')
	r = LIDARNode()

	rospy.spin()