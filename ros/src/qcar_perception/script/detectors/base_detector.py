from abc import ABCMeta, abstractmethod

import cv2
import numpy as np


class LaneDetector(object):
    __metaclass__ = ABCMeta
    COLOR_MAP = [[255, 0, 0],
                 [0, 255, 0],
                 [0, 0, 255],
                 [125, 125, 0],
                 [0, 125, 125],
                 [125, 0, 125],
                 [50, 100, 50],
                 [100, 50, 100]]

    @abstractmethod
    def get_lanes(self, img):
        pass

    @staticmethod
    def draw_lanes(lanes, shape):
        mask_image = np.zeros(shape=(shape[0], shape[1], 3), dtype=np.uint8)

        for i, xy_ in enumerate(lanes):
            color = LaneDetector.COLOR_MAP[i % len(LaneDetector.COLOR_MAP)]
            cv2.polylines(mask_image, xy_.reshape([-1, 1, 2]).astype(int), True, (color[0], color[1], color[2]))

        return mask_image
