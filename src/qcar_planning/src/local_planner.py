#!/usr/bin/env python3
from typing import List
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Bool
from visualization_msgs.msg import MarkerArray, Marker
from qcar_perception.msg import ObjectDetections2D, BoundingBox2D
from numpy import interp 

class LocalPlanningNode(object):
    def __init__(self):
        super().__init__()
        rospy.loginfo("Starting Planning System")
        self.subscribers()
        self.now = rospy.Time.now()
        self.throttle = 2
        self.steering = 0
        self.autonomous = False

        self.throttle_pub = rospy.Publisher('/qcar/velocity_target', Float64, queue_size=100)
        self.steering_pub = rospy.Publisher('/qcar/steering_target', Float64, queue_size=100)

    def subscribers(self):
        topic = '/planning/steering_delta'
        self._sub = rospy.Subscriber(topic, Float64, self.steering_callback, queue_size=1, buff_size=2**24)
        self.autonomous_pub = rospy.Subscriber('/qcar/autonomous_enabled', Bool, self.autonomous_callback)

    def steering_callback(self, data: Float64):
        if self.autonomous:
            data = data.data
            self.steering = interp(data, [-200, 200], [-0.25, 0.25])
            self.throttle_pub.publish(Float64(self.throttle))        
            self.steering_pub.publish(Float64(self.steering))

    def autonomous_callback(self, isEnabled):
        self.autonomous = isEnabled.data

if __name__ == '__main__':
    rospy.init_node('planning_node')
    r = LocalPlanningNode()
    rospy.spin()
