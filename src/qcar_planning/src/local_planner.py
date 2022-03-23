#!/usr/bin/env python3
import threading
from typing import List
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Float64MultiArray, Bool
from visualization_msgs.msg import MarkerArray, Marker
from qcar_perception.msg import ObjectDetections2D, BoundingBox2D
from qcar_perception.msg import BoundingBox, BoundingBoxes
from numpy import interp
from cv_bridge import CvBridge
from scipy.stats import linregress
import time

center_point = int(640 / 2)  # where is the camera center w.r.t the frame?
method = 1  # set to 2 for slope following instead.


class LocalPlanningNode(object):
    def __init__(self):
        super().__init__()
        rospy.loginfo("Starting Planning System")
        self.subscribers()
        self.now = rospy.Time.now()
        self.throttle = 9
        self.steering = 0
        self.bridge = CvBridge()
        self.autonomous = True
        self.offset_history = []
        self.turning = False
        self.elasped_time = 0
        self.last_stop_at_sign = None
        self.turn_time = rospy.get_rostime()

        self.commands = {
            "throttle": self.throttle,
            "steering": self.steering,
        }

        self.throttle_pub = rospy.Publisher(
            "/qcar/velocity_target", Float64, queue_size=100
        )
        self.steering_pub = rospy.Publisher(
            "/qcar/steering_target", Float64, queue_size=100
        )

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.autonomous:
                self.throttle_pub.publish(Float64(self.throttle))
                self.steering_pub.publish(Float64(self.steering))
            rate.sleep()

    def subscribers(self):
        waypoints_topic = "/planning/waypoints"
        center_topic = "/planning/center_offset"
        self._sub = rospy.Subscriber(
            waypoints_topic,
            Float64MultiArray,
            self.waypoints_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )
        self.autonomous_pub = rospy.Subscriber(
            "/qcar/autonomous_enabled", Bool, self.autonomous_callback
        )
        self.detected_objects_bounding_boxes = rospy.Subscriber(
            "/detected_objects_in_image",
            BoundingBoxes,
            self.detected_object_callback,
            queue_size=10,
        )

    def waypoints_callback(self, data):
        waypoints = data.data

        if method == 1:
            next_waypoint = np.mean(
                waypoints[0:250]
            )  # take the average of the next 100 waypoints
            offset = center_point - next_waypoint

            # if the offset is really small, just go straight
            if offset < 5:
                self.steering = 0

            else:
                self.steering = interp(offset, [-50, 50], [-1, 1])
            print(f"Offset: {offset}, Steer: {self.steering}")

        # Method 2 - Calculate slope of the waypoints curve and become parallel to it.
        if method == 2:
            result = linregress([i for i in range(0, len(waypoints))], waypoints)
            offset = result.slope * 1000
            self.steering = interp(offset, [-800, 800], [1, -1])
            print(f"Slope: {offset}, Steer: {self.steering}")

    def waypoints_callback_old(self, data: Float64MultiArray):
        data = data.data
        next_point = np.mean(data)
        center_point = 600
        offset = center_point - next_point
        print(offset)
        self.elasped_time = rospy.get_rostime() - self.turn_time

        if self.turning == False or (
            self.turning == True and self.elasped_time > rospy.Duration(2)
        ):
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

    def detected_object_callback(self, data: BoundingBoxes):
        depth_camera_data = rospy.wait_for_message("/camera/depth/image_raw", Image)
        cv_image = self.bridge.imgmsg_to_cv2(
            depth_camera_data, depth_camera_data.encoding
        )
        if self.last_stop_at_sign is None or (rospy.get_rostime() - self.last_stop_at_sign) > rospy.Duration(30):
            for box in data.bounding_boxes:
                if box.Class == "stop" and box.probability > 0.5:
                    pix = (
                        int(box.xmin + ((box.xmax - box.xmin) / 2)),
                        int(box.ymin + ((box.ymax - box.ymin) / 2)),
                    )
                    rospy.loginfo(
                        f"Detected Stop Sign at xmin: {box.xmin} ymin: {box.ymin} xmax: {box.xmax} ymax: {box.ymax}"
                    )
                    rospy.loginfo(
                        f"Average Depth Stop Sign Detection: {cv_image[pix[1], pix[0]]}"
                    )
                    if cv_image[pix[1], pix[0]] <= 600:
                        self.throttle = 0
                        if self.autonomous:
                            rospy.loginfo(f"Stopping at Stop Sign for 5 seconds")
                            self.throttle_pub.publish(Float64(self.throttle))
                            self.last_stop_at_sign = rospy.get_rostime()
                            time.sleep(5000)
                            self.throttle = 9
                            self.throttle_pub.publish(Float64(self.throttle))


if __name__ == "__main__":
    rospy.init_node("planning_node")
    r = LocalPlanningNode()
    rospy.spin()


