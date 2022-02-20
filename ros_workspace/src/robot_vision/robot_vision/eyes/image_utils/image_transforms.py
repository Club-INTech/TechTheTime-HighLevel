import cv2 as cv


def rotate(frame, rot: int):
    rows, cols = frame.shape
    rot_matrix = cv.getRotationMatrix2D(((cols - 1) / 2.0, (rows - 1) / 2.0), rot, 1)
    return cv.warpAffine(frame, rot_matrix, (cols, rows))


def color_transform(frame, lvls):
    return cv.cvtColor(frame, lvls)
