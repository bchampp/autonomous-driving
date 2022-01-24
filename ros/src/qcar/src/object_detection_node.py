#!/usr/bin/env python3
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from perception import ObjectDetector
from qcar.msg import ObjectDetection, ObjectDetections

class ObjectDetectionNode(object):
    def __init__(self):
        super().__init__()
        rospy.loginfo("Starting Object Detection Interface")
        self.subscribers()
        self.publishers()
        self.object_detector = ObjectDetector()
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
        now = rospy.get_rostime()

        image_np = self._cv_bridge.imgmsg_to_cv2(img_msg, 'bgr8')

        # perform object detections
        self.object_detector.detect_objects(image_np)

        if (len(self.object_detector.detections) > 0):
            detections_msg = self.convert_results_to_message(self.object_detector.detections)

            # TODO: get depth to object detections
            
            # visualize object detections
            image_np = ObjectDetector.overlay_detections(image_np, self.object_detector.detections)
            
            try:
                self.detections_pub.publish(detections_msg)
                self.visualize_pub.publish(self._cv_bridge.cv2_to_imgmsg(image_np, 'bgr8'))
                rospy.loginfo("Inference at %s FPS", 1.0/float(rospy.Time.now().to_sec() - self.now.to_sec()))
                self.now = rospy.Time.now()
            except Exception as e:
                rospy.logerr(e)

        else:
            msgs = ObjectDetections()
            self.detections_pub.publish(msgs)
        self.skip_count = 0

    def convert_results_to_message(self, detections):
        msgs = ObjectDetections()
        for i in range(len(detections)):
            msg = ObjectDetection()
            if detections[i].classification == 'stop sign':
                msg.classification = 0
            msg.confidence = detections[i].confidence
            msg.location = detections[i].center
            msgs.detections.append(msg)

        return msgs




if __name__ == '__main__':
    rospy.init_node('object_detection_node')
    r = ObjectDetectionNode()
    rospy.spin()