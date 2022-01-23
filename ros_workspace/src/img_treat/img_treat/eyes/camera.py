import cv2 as cv
import numpy as np

from template_matching_main import template_matching

template = cv.imread('/home/deffontaines/Images/Traitement_image/hexagone2.jpeg', 0)
print(type(template))

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
    #utilisation de la fonction template_matching :
    image = dst
    res = template_matching(image, template)
    # Display the resulting frame
    cv.imshow('frame', res)
    if cv.waitKey(1) == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()

import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
def template_matching(image, template):

    img_gray = image
    print(type(template))
    w, h = template.shape[::-1]

    res = cv.matchTemplate(img_gray,template,cv.TM_CCOEFF_NORMED)
    threshold = 0.8
    loc = np.where(res >= threshold)
    for pt in zip(*loc[::-1]):
        cv.rectangle(img_gray,pt, (pt[0] +w, pt[1] + h),(0,0,255),2)
    cv.imwrite('res.png',img_gray)
    return img_gray


#res2 = template_matching('/home/deffontaines/Images/Traitement_image/selena.jpeg','/home/deffontaines/Images/Traitement_image/mario1_template.jpg')
#cv.namedWindow('image',cv.WINDOW_NORMAL)
#cv.imshow('image',res2)
#cv.waitKey(0)