import robot_vision.eyes.eyes as eyes
import cv2 as cv
from robot_vision.eyes.exception.eyes_exception import EyesException


if __name__ == "__main__":
    c = eyes.Eyes(2, **{
        'known_distance': 20,
        'known_width': 5,
        'focal_measure': False,
        'distortion_calibration': True
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
