#!/usr/bin/env python3
from typing import List
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from perception import ObjectDetector, ObjectDetectionData
from sensor_msgs.msg import Image
from qcar_perception.msg import ObjectDetections2D, BoundingBox2D

class ObjectDetectionNode(object):
    def __init__(self):
        super().__init__()
        rospy.loginfo("Starting Object Detector")
        self.detector = ObjectDetector()
        rospy.loginfo("Initialized Object Detector")
        self.subscribers()
        self.publishers()
        self._cv_bridge = CvBridge()
        self.now = rospy.Time.now()

    def subscribers(self):
        topic = rospy.get_param('/realsense_camera_topic')
        self._sub = rospy.Subscriber(topic, Image, self.img_callback, queue_size=1, buff_size=2**24)

    def publishers(self):
        detection_topic = '/vision/yolo/detections'
        vis_topic = '/vision/yolo/visualization'
        self.detections_pub = rospy.Publisher(detection_topic, ObjectDetections2D, queue_size=10)
        self.visualize_pub = rospy.Publisher(vis_topic, Image, queue_size=1)

    def img_callback(self, img_msg):
        image_np = self._cv_bridge.imgmsg_to_cv2(img_msg, 'bgr8')

        try:
            self.detector.detect_objects(image_np)
            detections: List[ObjectDetectionData] = self.detector.detections

            if (len(detections) > 0):
                msg = ObjectDetections2D()

                # Create detection messages
                for detection in detections:
                    detection_msg = BoundingBox2D()
                    detection_msg.classification = detection.classification
                    detection_msg.probability = detection.confidence
                    msg.bounding_boxes.append(detection_msg)

                overlay_frame = self.detector.overlay_detections(image_np, detections)

                # Publish detection messages
                try:
                    self.detections_pub.publish(msg)
                    self.visualize_pub.publish(self._cv_bridge.cv2_to_imgmsg(overlay_frame, 'bgr8'))
                    rospy.loginfo("Inference at %s FPS", 1.0/float(rospy.Time.now().to_sec() - self.now.to_sec()))
                    self.now = rospy.Time.now()
                except Exception as e:
                    rospy.logerr(e)

            else:
                msgs = ObjectDetections2D()
                self.detections_pub.publish(msgs)
                self.visualize_pub.publish(self._cv_bridge.cv2_to_imgmsg(image_np, 'bgr8'))

        except Exception as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('vision_yolo_detection_node')
    r = ObjectDetectionNode()
    rospy.spin()
