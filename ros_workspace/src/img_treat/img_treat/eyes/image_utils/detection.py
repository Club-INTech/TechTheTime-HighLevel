import numpy as np
import cv2 as cv
import time

arucoDict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
arucoParams = cv.aruco.DetectorParameters_create()


cap = cv.VideoCapture(2)
if not cap.isOpened():
    print("Cannot open camera")
    exit()
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    #On retourne les images (rotation de 180°)
    rows,cols = gray.shape
    M = cv.getRotationMatrix2D(((cols-1)/2.0,(rows-1)/2.0),180,1)
    dst = cv.warpAffine(gray,M,(cols,rows))
    #use of Aruco
    (corners, ids, rejected) = cv.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
    print(corners)
    print(ids)
    # Display the resulting frame
    cv.imshow('frame', dst)
    if cv.waitKey(1) == ord('q'):
        break
    #time.sleep(1)
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()