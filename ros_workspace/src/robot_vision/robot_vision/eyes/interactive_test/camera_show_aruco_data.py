import robot_vision.eyes.eyes as eyes
import cv2 as cv
from robot_vision.eyes.exception.eyes_exception import EyesException


if __name__ == "__main__":
    c = eyes.Eyes(2, **{
        'color_transform': cv.COLOR_RGB2GRAY,
        'rotation': 180
    })
    while True:
        try:
            _, res = c.look("detect")
            print(res)
        except EyesException as e:
            print(e)
            continue
        if cv.waitKey(20) == ord('q'):
            break

    c.video_flow.release()
    cv.destroyAllWindows()
