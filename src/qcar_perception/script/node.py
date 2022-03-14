#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError

VERBOSE = True


class LaneDetectionNode:
    def __init__(self, sub, detector):
        self.image_pub = rospy.Publisher("/output/lane_detection/compressed", Image)
        self.subscriber = rospy.Subscriber(sub, Image, self.callback, queue_size=1)
        self.detector = detector
        self._cv_bridge = CvBridge()

    def callback(self, ros_data):
        print("Here!")
        image_np = self._cv_bridge.imgmsg_to_cv2(ros_data, 'bgr8')

        lanes, shape = self.detector.get_lanes(image_np)
        mask_image = self.detector.draw_lanes(lanes, shape)
        # Publish new image
        self.image_pub.publish(self._cv_bridge.cv2_to_imgmsg(mask_image, 'bgr8'))
