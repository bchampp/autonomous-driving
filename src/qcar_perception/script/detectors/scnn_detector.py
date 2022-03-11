import os

import numpy as np
import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()
from .base_detector import LaneDetector


class SCNNDetector(LaneDetector):
    _SHAPE = (288, 800)
    _MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pretrained-models", "scnn-model")

    def __init__(self, y_range):
        self.sess = tf.Session(graph=tf.Graph(), config=tf.ConfigProto(log_device_placement=False))
        tf.saved_model.loader.load(self.sess, ["serve"], self._MODEL_PATH)
        self.y_range = y_range

    @staticmethod
    def postprocess(cls, binary_seg_image, existence):
        lanes = []
        for ch in range(1, 5):
            channel = existence[ch - 1] * binary_seg_image[:, :, ch]
            y_ = range(0, 287)
            x, y = [], []
            for y__ in y_:
                if channel[y__].max() > 0.5:
                    y.append(y__)
                    x.append(channel[y__].argmax())

            if len(x) > 20:
                f1 = np.polyfit(y, x, 1)
                p1 = np.poly1d(f1)
                y_ = np.linspace(cls.y_range[0], cls.y_range[1], 100)
                x_ = p1(y_)
                xy_ = np.vstack((x_, y_)).transpose()
                lanes.append(xy_)
        return lanes, cls._SHAPE

    def get_lanes(self, img):
        binary_seg_image, existence = self.sess.run(["lanenet_loss_1/Squeeze:0", "lanenet_loss_1/Sigmoid:0"],
                                                    feed_dict={"input:0": [img]})

        return self.postprocess(self, binary_seg_image[0], existence[0])

    def __del__(self):
        pass
        # self.sess.close()
