#!/usr/bin/env python3
from typing import List
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Bool
from visualization_msgs.msg import MarkerArray, Marker
from qcar_perception.msg import ObjectDetections2D, BoundingBox2D
from qcar_perception.msg import BoundingBox, BoundingBoxes
from numpy import interp 
from cv_bridge import CvBridge


class LocalPlanningNode(object):
    def __init__(self):
        super().__init__()
        rospy.loginfo("Starting Planning System")
        self.subscribers()
        self.now = rospy.Time.now()
        self.throttle = 2
        self.steering = 0
        self.autonomous = False
        self.bridge = CvBridge()




        self.throttle_pub = rospy.Publisher('/qcar/velocity_target', Float64, queue_size=100)
        self.steering_pub = rospy.Publisher('/qcar/steering_target', Float64, queue_size=100)

    def subscribers(self):
        topic = '/planning/steering_delta'
        self._sub = rospy.Subscriber(topic, Float64, self.steering_callback, queue_size=1, buff_size=2**24)
        self.autonomous_pub = rospy.Subscriber('/qcar/autonomous_enabled', Bool, self.autonomous_callback)
        self.detected_objects_bounding_boxes = rospy.Subscriber('/detected_objects_in_image',BoundingBoxes,  self.detected_object_callback, queue_size=10)

        
        
    def steering_callback(self, data: Float64):
        if self.autonomous:
            data = data.data
            self.steering = interp(data, [-200, 200], [-0.25, 0.25])
            self.throttle_pub.publish(Float64(self.throttle))        
            self.steering_pub.publish(Float64(self.steering))

    def autonomous_callback(self, isEnabled):
        self.autonomous = isEnabled.data
        
    def detected_object_callback(self, data: BoundingBoxes):
        depth_camera_data = rospy.wait_for_message('/camera/depth/image_raw', Image)
        cv_image = self.bridge.imgmsg_to_cv2(depth_camera_data, depth_camera_data.encoding)
        for box in data.bounding_boxes:
               if box.Class == "stop" and box.probability > 0.5:
                        pix = (int(box.xmin+((box.xmax-box.xmin)/2)), int(box.ymin+((box.ymax-box.ymin)/2)))
                        rospy.loginfo(f"Detected Stop Sign at xmin: {box.xmin} ymin: {box.ymin} xmax: {box.xmax} ymax: {box.ymax}")
                        rospy.loginfo(f"Average Depth of Detection: {cv_image[pix[1], pix[0]]}")
                        if cv_image[pix[1], pix[0]] <= 600:
                              self.throttle_pub.publish(Float64(0))
                        elif cv_image[pix[1], pix[0]] > 600:
                              self.throttle_pub.publish(Float64(5))
    			

if __name__ == '__main__':
    rospy.init_node('planning_node')
    r = LocalPlanningNode()
    rospy.spin()
