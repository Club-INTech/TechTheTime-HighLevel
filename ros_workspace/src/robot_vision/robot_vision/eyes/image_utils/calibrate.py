from robot_vision.eyes.image_utils.detection import *
import math


def focal_length(frame, known_distance, known_width):
	res = detect(frame)
	if not res[1] is None:
		#_p = (math.fabs(get_coors(res)[0][0]-get_coors(res)[1][0]))
		_p = math.sqrt(
			(get_coors(res)[1][0] - get_coors(res)[0][0])**2
			+ (get_coors(res)[1][1] - get_coors(res)[0][1])**2)
		return (_p * known_distance) / known_width
	else:
		return None
