import time

import numpy as np
import cv2 as cv
import glob
cap = cv.VideoCapture(2)

def para_distort():
    
    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while len(imgpoints) < 10:
    #for fname in images:
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        #rows, cols = gray.shape
        #M = cv.getRotationMatrix2D(((cols - 1) / 2.0, (rows - 1) / 2.0), 180, 1)
        #dst = cv.warpAffine(gray, M, (cols, rows))
        # Find the chess board corners
        ret2, corners = cv.findChessboardCorners(gray, (7,6), None)
        # If found, add object points, image points (after refining them)
        if ret2 == True:
            objpoints.append(objp)
            corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners)
            # Draw and display the corners
            cv.drawChessboardCorners(frame, (7,6), corners2, ret)
            cv.imshow('img', frame)
            cv.waitKey(100)
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    """
        print("ret = ")
        print(ret)
        print("mtx = ")
        print(mtx)
        print("dist = ")
        print(dist)
        print("rvecs = ")
        print(rvecs)
        print("tvecs = ")
        print(tvecs)
    """
    cap.release()
    cv.destroyAllWindows()
    return [mtx, dist]

#mtx =[[2.05770829e+03,0.00000000e+00, 3.43848621e+02]
 #,[0.00000000e+00, 2.53983651e+03, 2.36558498e+02],
 #[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]

#dist =[[ 8.80556993e+00 -1.53231444e+02,  3.76651317e-02,  8.10566465e-02
  #-6.86844098e+02]]


def calibrated(frame):

    L= para_distort()

    mtx = L[0]
    dist = L[1]
    # print(mtx)
    # print(dist)

    cap = cv.VideoCapture(2)

    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while True:
        # for fname in images:
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        rows, cols = gray.shape
        M = cv.getRotationMatrix2D(((cols - 1) / 2.0, (rows - 1) / 2.0), 180, 1)
        dst = cv.warpAffine(gray, M, (cols, rows))
        h, w = frame.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

        # undistort
        dst = cv.undistort(frame, mtx, dist, None, newcameramtx)
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y + h, x:x + w]
        cv.imshow('la', dst)
        #cv.imshow('img', frame)
        if cv.waitKey(1) == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()


