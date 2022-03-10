from robot_vision.eyes.image_utils.detection import *
from robot_vision.eyes.image_utils.image_transforms import *
import cv2 as cv
import numpy as np
import math


def focal_calibration(frame, known_distance, known_width):
	res = detect(frame)
	if not res[1] is None:
		_p = math.sqrt(
			(get_coors(res)[1][0] - get_coors(res)[0][0])**2
			+ (get_coors(res)[1][1] - get_coors(res)[0][1])**2)
		return (_p * known_distance) / known_width
	else:
		return None


def distortion_calibration(frame, color, rotation, chessboard_form):

	transformed = rotate(color_transform(frame, color), rotation)
	ret, corners = cv.findChessboardCorners(transformed, chessboard_form, None)
	return ret, corners, transformed
