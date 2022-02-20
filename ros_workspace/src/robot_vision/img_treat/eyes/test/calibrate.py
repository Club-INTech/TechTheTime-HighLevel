import img_treat.eyes.eyes as eyes
import cv2 as cv
from img_treat.eyes.exception.eyes_exception import EyesException
import time
import math


if __name__ == "__main__":
    c = eyes.Eyes(2, **{
        'known_distance': 20,
        'known_width': 5,
        'focal_measure': True,
        'focal_length': 500
    })
    while True:
        try:
            _, res = c.look("calibrate")
            print(res)
        except EyesException as e:
            print(e)
            continue
        if cv.waitKey(1) == ord('q'):
            break

    c.video_flow.release()
    cv.destroyAllWindows()
