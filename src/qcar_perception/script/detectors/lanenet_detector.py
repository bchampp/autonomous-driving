import os

import cv2
import numpy as np
import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler

from .base_detector import LaneDetector


class LanenetLaneDetector(LaneDetector):
    DBSCAN_MIN_SAMPLES = 20
    DBSCAN_EPS = 0.35
    MORPHOLOGY_KERNEL = (5, 5)
    MIN_AREA_THR = 100
    _SHAPE = (256, 512)
    _MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pretrained-models", "lanenet-model")

    def __init__(self, y_range, dbscan_min_samples=DBSCAN_MIN_SAMPLES, dbscan_eps=DBSCAN_EPS,
                 morphology_kernel=MORPHOLOGY_KERNEL, min_area_thr=MIN_AREA_THR):
        self.sess = tf.Session(graph=tf.Graph(), config=tf.ConfigProto(log_device_placement=False))
        tf.saved_model.loader.load(self.sess, ["serve"], self._MODEL_PATH)
        self.y_range = y_range
        self.dbscan_min_samples = dbscan_min_samples
        self.dbscan_eps = dbscan_eps
        self.kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=morphology_kernel)
        self.min_area_thr = min_area_thr

    @staticmethod
    def morphology(cls, gray_image):
        gray_image = gray_image.astype(np.uint8)
        morphology_close = cv2.morphologyEx(gray_image, cv2.MORPH_CLOSE, cls.kernel, iterations=1)

        # Connected Component Analysis
        connect_components_analysis = cv2.connectedComponentsWithStats(morphology_close, connectivity=8,
                                                                       ltype=cv2.CV_32S)
        labels = connect_components_analysis[1]
        stats = connect_components_analysis[2]
        for index, stat in enumerate(stats):
            if stat[4] <= cls.min_area_thr:
                idx = np.where(labels == index)
                morphology_close[idx] = 0

        return morphology_close

    @staticmethod
    def clustering(cls, gray_image, instance_seg_image):
        # Clustering
        idx = (gray_image > 170).nonzero()
        lane_embedding_feats = instance_seg_image[idx]
        lane_coordinate = np.vstack((idx[1], idx[0])).transpose()

        db = DBSCAN(eps=cls.dbscan_eps, min_samples=cls.dbscan_min_samples)
        features = StandardScaler().fit_transform(lane_embedding_feats)
        db.fit(features)
        db_labels = db.labels_
        return lane_coordinate, db_labels

    @staticmethod
    def postprocess(cls, binary_seg_image, instance_seg_image):
        idx = binary_seg_image > 0.9
        gray_image = np.zeros_like(binary_seg_image)
        gray_image[idx] = 255
        morphology_close = cls.morphology(cls, gray_image)

        try:
            lane_coordinate, db_labels = cls.clustering(cls, morphology_close, instance_seg_image)
            unique_labels = np.unique(db_labels)
            num_clusters = len(unique_labels)
            assert num_clusters > 0
        except:
            return [], cls._SHAPE

        min_y, max_y = cls.y_range

        lanes = []
        for index, label in enumerate(unique_labels.tolist()):
            if label == -1:
                continue
            idx = np.where(db_labels == label)
            xy = lane_coordinate[idx]
            x = xy[:, 0]
            y = xy[:, 1]
            f1 = np.polyfit(y, x, 1)
            p1 = np.poly1d(f1)
            y_ = np.linspace(min_y, max_y, 50)
            x_ = p1(y_)
            xy_ = np.vstack((x_, y_)).transpose()
            lanes.append(xy_)

        return lanes, cls._SHAPE

    def get_lanes(self, img):
        binary_seg_image, instance_seg_image = self.sess.run(["lanenet_model/ArgMax:0",
                                                              "lanenet_model/pix_embedding_relu:0"],
                                                             feed_dict={"input:0": [img]})

        return self.postprocess(self, binary_seg_image[0], instance_seg_image[0])

    def __del__(self):
        pass
        # self.sess.close()