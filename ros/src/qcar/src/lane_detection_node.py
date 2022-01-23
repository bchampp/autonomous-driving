#!/usr/bin/env python3
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from perception import LaneDetector

class ObjectDetectionNode(object):
    def __init__(self):
        super().__init__()
        rospy.loginfo("Starting Lane Detection Interface")
        self.subscribers()
        self.publishers()
        self.lane_detector = LaneDetector()
        self._cv_bridge = CvBridge()
        self.now = rospy.Time.now()

    def subscribers(self):
        topic = '/qcar/rgbd_color'
        self._sub = rospy.Subscriber(topic, Image, self.img_callback, queue_size=1, buff_size=2**24)

    def publishers(self):
        topic = '/qcar/object_detections'
        self._pub = rospy.Publisher(topic, Image, queue_size=0)

    def img_callback(self, img_msg):
        now = rospy.get_rostime()
        image_np = self._cv_bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        self.object_detector.detect_lanes(image_np)
        
        try:
            self._pub.publish(self._cv_bridge.cv2_to_imgmsg(image_np, 'bgr8'))
            rospy.loginfo("Inference at %s FPS", 1.0/float(rospy.Time.now().to_sec() - self.now.to_sec()))
            self.now = rospy.Time.now()
        except Exception as e:
            rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node('object_detection_node')
    r = ObjectDetectionNode()
    rospy.spin()