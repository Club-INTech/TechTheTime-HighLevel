import time
import cv2 as cv
from typing import Tuple, Any
from img_treat.eyes.exception.cannot_receive_frame_exception import CannotReceiveFrameException
from img_treat.eyes.exception.cannot_open_camera_exception import CannotOpenCameraException
from img_treat.eyes.exception.parameter_is_not_set_exception import ParameterIsNotSetException
from img_treat.eyes.image_utils.image_transforms import *
from img_treat.eyes.image_utils.detection import *
from img_treat.eyes.image_utils.distance import *
from img_treat.eyes.image_utils.calibrate import focal_length


class Eyes:
    def __init__(self, capture_num: int, **config) -> None:
        self.video_flow = cv.VideoCapture(capture_num)
        for key, value in config.items():
            setattr(self, key, value)

    def look(self, request_type: str) -> Tuple[int, Any]:
        if not self.video_flow.isOpened():
            raise CannotOpenCameraException("line 11 in eyes.py: self.video_flow.isOpened()")
        ret, frame = self.video_flow.read()
        if not ret:
            raise CannotReceiveFrameException("line 14 in eyes.py: ret, frame = self.video_flow.read()")
        return time.time_ns(), getattr(Eyes, '_Eyes__' + request_type).__call__(self, frame)

    def __check_attr_list(self, attr_name_list: list) -> None:
        for attr_name in attr_name_list:
            if not self.__dict__.__contains__(attr_name):
                raise ParameterIsNotSetException(attr_name)

    def __show(self, frame) -> bool:
        self.__check_attr_list(list(["color_transform", "rotation"]))
        dst = rotate(color_transform(frame, self.color_transform), self.rotation)
        cv.imshow('frame', dst)
        return True

    def __detect(self, frame) -> list:
        self.__check_attr_list(list(["color_transform", "rotation"]))
        res = detect(frame)
        self.__show(frame)
        return res

    def __distance(self, frame) -> float:
        self.__check_attr_list(list(["color_transform", "rotation"]))
        res = distance_to_camera(frame)
        return res

    # def __calibrate(self, frame) -> float:
    #     self.__check_attr_list(list(["known_distance", "known_width", 'focal_measure', 'focal_length']))
    #     if self.focal_measure:
    #         return distance_to_camera(frame, self.known_width, self.focal_length)
    #     else:
    #         return focal_length(frame, self.known_distance, self.known_width)

    def __calibrate(self, frame) -> float:
        self.__check_attr_list(list(["known_distance", "known_width", 'focal_measure', 'focal_length']))
        if self.focal_measure:
            return distance_to_camera(frame, self.known_width, self.focal_length)
        else:
            return focal_length(frame, self.known_distance, self.known_width)
