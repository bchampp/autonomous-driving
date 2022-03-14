#!/usr/bin/env python3

import argparse
import os
import sys
from os import path

# Set Python Path and Current Working Directory for Python
DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(DIR)
os.chdir(DIR)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Lane Detection")
    parser.add_argument("--detector", dest="detector", help="lanenet/scnn", default="lanenet", type=str)
    parser.add_argument("--subscriber", dest="subscriber", help="video/node", default='node', type=str)
    parser.add_argument("--name", dest="name", help="path to video or the node to be subscribed")
    parsed_args = parser.parse_args()

    assert parsed_args.detector.lower() in ["lanenet", "scnn"]
    assert parsed_args.subscriber.lower() in ["node", "video"]

    if parsed_args.detector.lower() == "lanenet":
        from detectors.lanenet_detector import LanenetLaneDetector
        detector = LanenetLaneDetector(y_range=[151, 256])
    else:
        from detectors.scnn_detector import SCNNDetector
        detector = SCNNDetector(y_range=[180, 288])

    if parsed_args.subscriber.lower() == "video":
        import cv2
        fp = path.join(path.dirname(__file__), 'highway.mp4')
        print(fp)
        cap = cv2.VideoCapture(fp)
        while cap.isOpened():
            print("!")
            ret, image = cap.read()
            lanes, shape = detector.get_lanes(image)
            mask_image = detector.draw_lanes(lanes, shape)
            cv2.imshow('frame', mask_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                exit()
        cap.release()

    else:
        import rospy
        from node import LaneDetectionNode
        lane_detector_node = LaneDetectionNode(parsed_args.name, detector)
        print("Hello world!")
        rospy.init_node('Lane_Detector', anonymous=True)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down ROS Lane Detector Module")
