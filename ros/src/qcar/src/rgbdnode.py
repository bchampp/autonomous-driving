#!/usr/bin/env python3

from __future__ import division, print_function, absolute_import

import roslib
import rospy
import numpy as np
import cv2
from qcar.q_essential import Camera3D

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

class RGBDNode(object):
	def __init__(self):
		super().__init__()
		self.rgbd_color_pub = rospy.Publisher('/qcar/realsense_color', Image, queue_size=1)
		self.rgbd_depth_pub = rospy.Publisher('/qcar/realsense_depth', Image, queue_size=1)
		self.bridge = CvBridge()
		
		#Initialize CV sensors
		rgbd = Camera3D(mode='RGB&DEPTH', frame_width_RGB=1280, frame_height_RGB=720)

		#Get readings
		while not rospy.is_shutdown():
			rgbd.read_RGB()
			rgbd.read_depth(dataMode='m')
			
			# self.rate_pub.publish(msg)
			self.process_color_data(self.rgbd_color_pub, rgbd.image_buffer_RGB)
			self.process_depth_data(self.rgbd_depth_pub, rgbd.image_buffer_depth_m)

		rgbd.terminate()

#--------------------------------------------------------------------------------------------------------------
	def process_color_data(self, cam_info, img_data):

		pub_img = self.bridge.cv2_to_imgmsg(img_data, "bgr8")
		pub_img.header.stamp =  rospy.Time.now()
		pub_img.header.frame_id = 'RGBD_color_input'
		cam_info.publish(pub_img)

	def process_depth_data(self, cam_info, img_data):

		pub_img = self.bridge.cv2_to_imgmsg(img_data, "32FC1")
		pub_img.header.stamp =  rospy.Time.now()
		pub_img.header.frame_id = 'RGBD_depth_input'
		cam_info.publish(pub_img)

if __name__ == '__main__':
	rospy.init_node('rgbd_node')
	r = RGBDNode()

	rospy.spin()
		