#!/usr/bin/env python3
from typing import List
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from lanedetection import LaneDetector
from sensor_msgs.msg import Image
from qcar_perception.msg import ObjectDetections2D, BoundingBox2D

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
        self.detector.set_constants(640, 480)
        self.detector.set_roi(simulation_roi)
        rospy.loginfo("Initialized Lane Detector")
        self.subscribers()
        self.publishers()
        self._cv_bridge = CvBridge()
        self.now = rospy.Time.now()

    def subscribers(self):
        topic = '/camera/color/image_raw'
        self._sub = rospy.Subscriber(topic, Image, self.img_callback, queue_size=1, buff_size=2**24)

    def publishers(self):
        detection_topic = '/vision/lanes/detections'
        vis_topic = '/vision/yolo/visualization'
        self.detections_pub = rospy.Publisher(detection_topic, ObjectDetections2D, queue_size=10)
        self.visualize_pub = rospy.Publisher(vis_topic, Image, queue_size=1)

    def img_callback(self, img_msg):
        image_np = self._cv_bridge.imgmsg_to_cv2(img_msg, 'bgr8')

        try:
            self.detector.detect_lanes(image_np)
            overlay = self.detector.overlay_detections(image_np)
            
            self.detector.plot_roi(image_np)
            cv2.imshow("Test", self.detector.warped_frame)
            cv2.imshow("Lane Detections", overlay)
            cv2.waitKey(1)

            # if (len(detections) > 0):
            #     msg = ObjectDetections2D()

            #     # Create detection messages
            #     for detection in detections:
            #         detection_msg = BoundingBox2D()
            #         detection_msg.classification = detection.classification
            #         detection_msg.probability = detection.confidence
            #         msg.bounding_boxes.append(detection_msg)

            #     overlay_frame = self.detector.overlay_detections(image_np, detections)

            #     # Publish detection messages
            #     try:
            #         self.detections_pub.publish(msg)
            #         self.visualize_pub.publish(self._cv_bridge.cv2_to_imgmsg(overlay_frame, 'bgr8'))
            #         rospy.loginfo("Inference at %s FPS", 1.0/float(rospy.Time.now().to_sec() - self.now.to_sec()))
            #         self.now = rospy.Time.now()
            #     except Exception as e:
            #         rospy.logerr(e)

            # else:
            #     msgs = ObjectDetections2D()
            #     self.detections_pub.publish(msgs)
            #     self.visualize_pub.publish(self._cv_bridge.cv2_to_imgmsg(image_np, 'bgr8'))

        except Exception as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('vision_lane_detection_node')
    r = LaneDetectionNode()
    rospy.spin()
