import cv2 as cv
from img_treat.eyes.image_utils.image_transforms import *


def detect(frame):
    arucoDict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
    arucoParams = cv.aruco.DetectorParameters_create()
    res = []
        # if frame is read correctly ret is True
        # Our operations on the frame come here
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        #On retourne les images (rotation de 180°)
        #rows,cols = gray.shape
        #M = cv.getRotationMatrix2D(((cols-1)/2.0,(rows-1)/2.0),180,1)
        #dst = cv.warpAffine(gray,M,(cols,rows))
        #dst = rotate(color_transform(frame, self.color_transform), self.rotation)
        #use of Aruco
    (corners, ids, rejected) = cv.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)
    res.append(corners)
    res.append(ids)
        # Display the resulting frame
        #cv.imshow('frame', dst)
        #if cv.waitKey(1) == ord('q'):
            #break
    return res
        #time.sleep(1)
    # When everything done, release the capture
    #cap.release()
    #cv.destroyAllWindows()