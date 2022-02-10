#!/usr/bin/env python3
import roslib
import rospy
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from qcar.msg import ObjectDetection, ObjectDetections
from object_detection.object_detector_detection_api import ObjectDetectorDetectionAPI
import cv2
import threading

class ObjectDetectionNode(object):
    def __init__(self):
        super().__init__()
        rospy.loginfo("Starting Object Detection Interface")
        curr_dir = os.path.dirname(os.path.abspath(__file__))
        self.predictor = ObjectDetectorDetectionAPI()
        self.subscribers()
        self.publishers()
        self._cv_bridge = CvBridge()
        self.now = rospy.Time.now()

    def subscribers(self):
        topic = '/qcar/realsense_color'
        self._sub = rospy.Subscriber(topic, Image, self.img_callback, queue_size=1, buff_size=2**24)

    def publishers(self):
        detection_topic = '/perception/object_detections'
        vis_topic = '/perception/object_visualization'
        self.detections_pub = rospy.Publisher(detection_topic, ObjectDetections, queue_size=10)
        self.visualize_pub = rospy.Publisher(vis_topic, Image, queue_size=1)

    def img_callback(self, img_msg):
        image_np = self._cv_bridge.imgmsg_to_cv2(img_msg, 'bgr8')

        try:
            # TODO: This can be better
            result = self.predictor.detect(image_np)

            if (len(result) > 0):
                msg = ObjectDetections()

                for obj in result:
                    detection = ObjectDetection()
                    if obj[3] == 'stop sign':
                        detection.classification = 0
                    else:
                        detection.classification = -1
                    detection.confidence = obj[2]
                    detection.location = obj[0]
                    msg.detections.append(detection)
                    rospy.loginfo('coordinates: {} {}. class: "{}". confidence: {:.2f}'.
                            format(obj[0], obj[1], obj[3], obj[2]))
                    cv2.rectangle(image_np, obj[0], obj[1], (0, 255, 0), 2)
                    cv2.putText(image_np, '{}: {:.2f}'.format(obj[3], obj[2]),
                        (obj[0][0], obj[0][1] - 5), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)

                try:
                    self.detections_pub.publish(msg)
                    self.visualize_pub.publish(self._cv_bridge.cv2_to_imgmsg(image_np, 'bgr8'))
                    rospy.loginfo("Inference at %s FPS", 1.0/float(rospy.Time.now().to_sec() - self.now.to_sec()))
                    self.now = rospy.Time.now()
                except Exception as e:
                    rospy.logerr(e)

            else:
                msgs = ObjectDetections()
                self.detections_pub.publish(msgs)
                self.visualize_pub.publish(self._cv_bridge.cv2_to_imgmsg(image_np, 'bgr8'))

        except Exception as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('object_detection_node')
    r = ObjectDetectionNode()
    rospy.spin()