# 1) Find distance from camera to palet(puck) :

# import the necessary packages
from imutils import paths
import numpy as np
import imutils
import cv2
from img_treat.eyes.image_utils.detection import *
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


# loop over the images
# for imagePath in sorted(paths.list_images("images")):
# 	# load the image, find the marker in the image, then compute the
# 	# distance to the marker from the camera
# 	image = cv2.imread(imagePath)
# 		# draw a bounding box around the image and display it
# 		box = cv2.cv.BoxPoints(get_coors(image)) if imutils.is_cv2() else cv2.boxPoints(get_coors(image))
# 		box = np.int0(box)
# 		cv2.drawContours(image, [box], -1, (0, 255, 0), 2)
# 		cv2.putText(image, "%.2fft" % (inches / 12), (image.shape[1] - 200, image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 3)
# 		cv2.imshow("image", image)
# 		cv2.waitKey(0)