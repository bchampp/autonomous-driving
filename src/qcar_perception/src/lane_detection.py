#!/usr/bin/env python3
from typing import List
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from lanedetection import LaneDetector
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray, Marker
from qcar_perception.msg import ObjectDetections2D, BoundingBox2D

SIMULATION = True
simulation_roi = np.float32([
		(200, 250),  # Top-left corner
		(-250, 480),  # Bottom-left corner            
		(900, 480), # Bottom-right corner
		(550, 250)   # Top-right corner
	])

def average(image, lines):
    left = []
    right = []
    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        #fit line to points, return slope and y-int
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        y_int = parameters[1]
        #lines on the right have positive slope, and lines on the left have neg slope
        if slope < 0:
            left.append((slope, y_int))
        else:
            right.append((slope, y_int))
    #takes average among all the columns (column0: slope, column1: y_int)
    right_avg = np.average(right, axis=0)
    left_avg = np.average(left, axis=0)
    #create lines based on averages calculates
    left_line = make_points(image, left_avg)
    right_line = make_points(image, right_avg)
    return np.array([left_line, right_line])

def make_points(image, average):
    slope, y_int = average
    y1 = image.shape[0]
    #how long we want our lines to be --> 3/5 the size of the image
    y2 = int(y1 * (3/5))
    #determine algebraically
    x1 = int((y1 - y_int) // slope)
    x2 = int((y2 - y_int) // slope)
    return np.array([x1, y1, x2, y2])
    
def display_lines(image, lines):
    lines_image = np.zeros_like(image)
    #make sure array isn't empty
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line
            #draw lines on a black image
            cv2.line(lines_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
    return lines_image
    
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
        try:
            topic = rospy.get_param('/front_camera_topic')
        except KeyError as e:
            topic = "/qcar/csi_front"
        self._sub = rospy.Subscriber(topic, Image, self.img_callback, queue_size=1, buff_size=2**24)

    def publishers(self):
        vis_camera_topic = '/vision/lanes/detections'
        vis_top_topic = '/vision/lanes/detections_top'
        vis_markers_topic = '/vision/lanes/markers'
        self.visualize_camera_pub = rospy.Publisher(vis_camera_topic, Image, queue_size=1)
        self.visualize_top_pub = rospy.Publisher(vis_top_topic, Image, queue_size=1)
        self.visualize_markers = rospy.Publisher(vis_markers_topic, MarkerArray, queue_size=1)

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

        try:
            self.detector.detect_lanes(image_np)
            camera_overlay = self.detector.overlay_detections(image_np)
            top_overlay = self.detector.lanes_top_view
            self.visualize_camera_pub.publish(self._cv_bridge.cv2_to_imgmsg(camera_overlay, 'bgr8'))
            self.visualize_top_pub.publish(self._cv_bridge.cv2_to_imgmsg(top_overlay, 'bgr8'))
            self.visualize_markers.publish(self.create_markers(self.detector.lane_pts_top_view))
        except Exception as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('vision_lane_detection_node')
    r = LaneDetectionNode()
    rospy.spin()
