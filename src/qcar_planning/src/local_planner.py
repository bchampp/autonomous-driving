#!/usr/bin/env python3
from typing import List
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Float64MultiArray, Bool
from visualization_msgs.msg import MarkerArray, Marker
from qcar_perception.msg import ObjectDetections2D, BoundingBox2D
from numpy import interp 

center_point = int(1280 / 2) # where is the camera center w.r.t the frame?

class LocalPlanningNode(object):
    def __init__(self):
        super().__init__()
        rospy.loginfo("Starting Planning System")
        self.subscribers()
        self.now = rospy.Time.now()
        self.throttle = 10
        self.steering = 0
        self.autonomous = True
        self.offset_history = []
        self.turning = False
        self.elasped_time = 0
        self.turn_time = rospy.get_rostime()

        self.throttle_pub = rospy.Publisher('/qcar/velocity_target', Float64, queue_size=100)
        self.steering_pub = rospy.Publisher('/qcar/steering_target', Float64, queue_size=100)

    def subscribers(self):
        topic = '/planning/waypoints'
        self._sub = rospy.Subscriber(topic, Float64MultiArray, self.waypoints_callback, queue_size=1, buff_size=2**24)
        self.autonomous_pub = rospy.Subscriber('/qcar/autonomous_enabled', Bool, self.autonomous_callback)

    def waypoints_callback(self, data):
        waypoints = data.data
        next_waypoint = np.mean(waypoints[0:200]) # take the average of the next 100 waypoints
        offset = center_point - next_waypoint
        print(offset)

        # irrelevant offsets just drive straight
        if abs(offset) < 20:
            self.steering = interp(offset, [-100, 100], [-1, 1])

        else:
            self.steering = interp(offset, [-20, 20], [-1, 1])

        print(self.steering)

        if self.autonomous:
            self.throttle_pub.publish(Float64(self.throttle))        
            self.steering_pub.publish(Float64(self.steering))

    def waypoints_callback_old(self, data: Float64MultiArray):
        data = data.data
        next_point = np.mean(data)
        center_point = 600
        offset = center_point - next_point
        print(offset)
        self.elasped_time = rospy.get_rostime() - self.turn_time
        

        if self.turning == False or (self.turning == True and self.elasped_time > rospy.Duration(2)): 
            # not a big offset, just drive forwards. 
            if abs(offset) < 15:
                self.steering = 0

            # in a corner, make steering more aggressive and increase window size. 
            elif abs(offset) > 20:
                next_point = np.mean(data[100:720])
                offset = center_point - next_point
                self.steering = interp(offset, [-25, 25], [-1, 1])
                if self.steering > 1:
                    self.steering = 1

                if self.steering < -1:
                    self.steering = -1

                self.turning = True
                self.turn_time = rospy.get_rostime()

            # in the middle
            else:
                self.steering = interp(offset, [-100, 100], [-1, 1])


        # if len(self.offset_history) > 5:
            # del self.offset_history[0]
        
        # self.offset_history.append(offset)
        # offset_average = np.mean(self.offset_history)

        # self.steering = interp(offset_average, [-100, 100], [-1, 1])
        
        if self.autonomous:
            self.throttle_pub.publish(Float64(self.throttle))        
            self.steering_pub.publish(Float64(self.steering))

    def autonomous_callback(self, isEnabled):
        self.autonomous = isEnabled.data

if __name__ == '__main__':
    rospy.init_node('planning_node')
    r = LocalPlanningNode()
    rospy.spin()
