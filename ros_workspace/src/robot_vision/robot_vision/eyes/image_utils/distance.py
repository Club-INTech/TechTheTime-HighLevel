from robot_vision.eyes.image_utils.detection import *
import math


def distance_to_camera(frame, known_width, focal_length):
	res = detect(frame)
	if not res[1] is None:
		_p = math.sqrt(
			(get_coors(res)[1][0] - get_coors(res)[0][0])**2
			+ (get_coors(res)[1][1] - get_coors(res)[0][1])**2)
		return (known_width * focal_length) / _p
	else:
		return -1