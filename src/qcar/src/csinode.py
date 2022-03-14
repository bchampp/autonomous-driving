#!/usr/bin/env python3

from __future__ import division, print_function, absolute_import

import roslib
import rospy
import numpy as np
import cv2
from qcar.q_essential import Camera2D

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

class CSINode(object):
	def __init__(self):
		super().__init__()
		self.imageWidth = 640
		self.imageHeight = 480
		self.sampleRate = 30.0
		self.cam_pub_r = rospy.Publisher('/qcar/csi_right', Image, queue_size=10)
		self.cam_pub_b = rospy.Publisher('/qcar/csi_back', Image, queue_size=10)
		self.cam_pub_l = rospy.Publisher('/qcar/csi_left', Image, queue_size=10)
		self.cam_pub_f = rospy.Publisher('/qcar/csi_front', Image, queue_size=10)
		self.bridge = CvBridge()

		csi1 = Camera2D(camera_id="0", frame_width=self.imageWidth, frame_height=self.imageHeight, frame_rate=self.sampleRate)
		csi2 = Camera2D(camera_id="1", frame_width=self.imageWidth, frame_height=self.imageHeight, frame_rate=self.sampleRate)
		csi3 = Camera2D(camera_id="2", frame_width=self.imageWidth, frame_height=self.imageHeight, frame_rate=self.sampleRate)
		csi4 = Camera2D(camera_id="3", frame_width=self.imageWidth, frame_height=self.imageHeight, frame_rate=self.sampleRate)

		while not rospy.is_shutdown():
			csi1.read()
			csi2.read()
			csi3.read()
			csi4.read()
			
			# self.rate_pub.publish(msg)
			self.process_cam_data(self.cam_pub_r, csi1.image_data)
			self.process_cam_data(self.cam_pub_b, csi2.image_data)
			self.process_cam_data(self.cam_pub_l, csi3.image_data)
			self.process_cam_data(self.cam_pub_f, csi4.image_data)

		csi1.terminate()
		csi2.terminate()
		csi3.terminate()
		csi4.terminate()

#--------------------------------------------------------------------------------------------------------------
	def process_cam_data(self, cam_info, img_data):

		pub_img = self.bridge.cv2_to_imgmsg(img_data, "bgr8")
		pub_img.header.stamp =  rospy.Time.now() 
		pub_img.header.frame_id = 'cam_img_input'
		# cv2.imshow(str(cam_info), img_data)
		# cv2.waitKey(1)
		cam_info.publish(pub_img)

if __name__ == '__main__':
	rospy.init_node('csi_node')
	r = CSINode()

	rospy.spin()
		