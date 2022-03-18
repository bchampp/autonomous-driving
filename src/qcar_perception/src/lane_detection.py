#!/usr/bin/env python3
from typing import List
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from lanedetection import LaneDetector
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from visualization_msgs.msg import MarkerArray, Marker
from qcar_perception.msg import ObjectDetections2D, BoundingBox2D

simulation_roi = np.float32([
		(0, 500),  # Top-left corner
		(0, 720),  # Bottom-left corner            
		(1280, 720), # Bottom-right corner
		(1280, 500)   # Top-right corner
	])

real_roi = np.float32([
    (200, 250),  # Top-left corner
    (-600, 480),  # Bottom-left corner            
    (1240, 480), # Bottom-right corner
    (440, 250)   # Top-right corner
])

class LaneDetectionNode(object):
    def __init__(self):
        super().__init__()
        rospy.loginfo("Starting Lane Detector")
        self.detector = LaneDetector()
        if (rospy.get_param('is_simulation')):
            self.detector.set_constants(1280, 720)
            self.detector.set_roi(simulation_roi)
        else:
            self.detector.set_constants(640, 480)
            self.detector.set_roi(real_roi)
        rospy.loginfo("Initialized Lane Detector")
        self.subscribers()
        self.publishers()
        self._cv_bridge = CvBridge()
        self.now = rospy.Time.now()

    def subscribers(self):
        try:
            topic = rospy.get_param('/front_camera_topic')
        except KeyError as e:
            topic = "/qcar/csi_front"
        self._sub = rospy.Subscriber(topic, Image, self.img_callback, queue_size=1, buff_size=2**24)

    def publishers(self):
        vis_camera_topic = '/vision/lanes/detections'
        vis_top_topic = '/vision/lanes/detections_top'
        vis_markers_topic = '/vision/lanes/markers'
        vis_planning_topic = '/planning/steering_delta'
        self.visualize_camera_pub = rospy.Publisher(vis_camera_topic, Image, queue_size=1)
        self.visualize_top_pub = rospy.Publisher(vis_top_topic, Image, queue_size=1)
        self.visualize_markers = rospy.Publisher(vis_markers_topic, MarkerArray, queue_size=1)
        self.planning_pub = rospy.Publisher(vis_planning_topic, Float64, queue_size=1)

    def create_markers(self, pts):
        marker_array = MarkerArray()
        for p in pts[0]:
            marker = Marker()
            marker.header.frame_id = "lanes"
            marker.type = marker.POINTS
            marker.action = marker.ADD
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 1
            marker.color.a = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = p[1]
            marker.pose.position.y = p[0]
            marker.pose.position.z = 0
            marker_array.markers.append(marker)
        
        id = 0
        for m in marker_array.markers:
            m.id = id
            id += 1
        return marker_array

    def img_callback(self, img_msg):
        image_np = self._cv_bridge.imgmsg_to_cv2(img_msg, 'bgr8')

        # TODO: Dynamically set the ROI
        try:
            self.detector.detect_lanes(image_np)
            camera_overlay = self.detector.overlay_detections(image_np)
            self.detector.plot_roi(image_np)
            cv2.waitKey(1)
            top_overlay = self.detector.lanes_top_view
            self.visualize_camera_pub.publish(self._cv_bridge.cv2_to_imgmsg(camera_overlay, 'bgr8'))
            self.visualize_top_pub.publish(self._cv_bridge.cv2_to_imgmsg(top_overlay, 'bgr8'))
            self.visualize_markers.publish(self.create_markers(self.detector.lane_pts_top_view))
            self.planning_pub.publish((image_np.shape[1] / 2) - self.detector.target_x)
        except Exception as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('vision_lane_detection_node')
    r = LaneDetectionNode()
    rospy.spin()
