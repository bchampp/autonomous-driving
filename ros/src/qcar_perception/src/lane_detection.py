#!/usr/bin/env python3
from typing import List
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from lanedetection import LaneDetector
from sensor_msgs.msg import Image
from qcar_perception.msg import ObjectDetections2D, BoundingBox2D

SIMULATION = False
simulation_roi = np.float32([
		(0, 250),  # Top-left corner
		(-100, 480),  # Bottom-left corner            
		(800, 480), # Bottom-right corner
		(640, 250)   # Top-right corner
	])



class LaneDetectionNode(object):
    def __init__(self):
        super().__init__()
        rospy.loginfo("Starting Lane Detector")
        self.detector = LaneDetector()
        if (SIMULATION):
            self.detector.set_constants(640, 480)
            self.detector.set_roi(simulation_roi)
        rospy.loginfo("Initialized Lane Detector")
        self.subscribers()
        self.publishers()
        self._cv_bridge = CvBridge()
        self.now = rospy.Time.now()

    def subscribers(self):
        topic = '/qcar/realsense_color'
        self._sub = rospy.Subscriber(topic, Image, self.img_callback, queue_size=1, buff_size=2**24)

    def publishers(self):
        detection_topic = '/vision/lanes/detections'
        vis_topic = '/vision/lanes/visualization'
        self.detections_pub = rospy.Publisher(detection_topic, ObjectDetections2D, queue_size=10)
        self.visualize_pub = rospy.Publisher(vis_topic, Image, queue_size=1)

    def img_callback(self, img_msg):
        image_np = self._cv_bridge.imgmsg_to_cv2(img_msg, 'bgr8')

        try:
            self.detector.detect_lanes(image_np)
            overlay = self.detector.overlay_detections(image_np)
            cv2.imshow("Lane Detections", overlay)
            cv2.waitKey(1)

            self.visualize_pub.publish(self._cv_bridge.cv2_to_imgmsg(overlay, 'bgr8'))

        except Exception as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('vision_lane_detection_node')
    r = LaneDetectionNode()
    rospy.spin()
