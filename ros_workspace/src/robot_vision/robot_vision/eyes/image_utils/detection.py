import cv2 as cv
from robot_vision.eyes.image_utils.image_transforms import *


def detect(frame):
    arucoDict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
    arucoParams = cv.aruco.DetectorParameters_create()
    res = []
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    (corners, ids, rejected) = cv.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)
    res.append(corners)
    res.append(ids)
    return res

def get_coors(detect_output):
    return detect_output[0][0][0]
