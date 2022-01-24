#!/usr/bin/env python3
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from perception import ObjectDetector
from qcar.msg import ObjectDetection, ObjectDetections, MotorCommands
import time

class PlanningNode(object):
    def __init__(self):
        super().__init__()
        rospy.loginfo("Starting Planner")
        self.subscribers()
        self.publishers()
        self._cv_bridge = CvBridge()
        self.now = rospy.Time.now()
        self.throttle = 0
        self.steering = 0.0
        self.sample_time = 0.001
        self.stopped = False

    def looping(self):
        while not rospy.is_shutdown():
            msg = MotorCommands()
            msg.throttle = self.throttle
            msg.steering = self.steering
            self.control_pub.publish(msg)
            time.sleep(self.sample_time)

    def subscribers(self):
        topic = '/perception/object_detections'
        self._sub = rospy.Subscriber(topic, ObjectDetections, self.object_callback, queue_size=1, buff_size=2**24)

    def publishers(self):
        control_topic = '/qcar/control'
        self.control_pub = rospy.Publisher(control_topic, MotorCommands, queue_size=0)

    def object_callback(self, object):
        now = rospy.get_rostime()

        for i in range(len(object.detections)):
            if (object.detections[i].classification == 0):
                self.stop_time = rospy.Time.now()
                self.throttle = 0
                self.stopped = True

            else:
                self.throttle = 0.1

if __name__ == '__main__':
    rospy.init_node('planning_node')
    r = PlanningNode()
    r.looping()
    rospy.spin()