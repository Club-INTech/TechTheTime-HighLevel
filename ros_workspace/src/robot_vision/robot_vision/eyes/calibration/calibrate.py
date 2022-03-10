import robot_vision.eyes.eyes as eyes
import cv2 as cv
from robot_vision.eyes.exception.eyes_exception import EyesException
import numpy as np

def focal_calibration(c: eyes.Eyes):
    focal_lengths = []
    i = 0
    while i < 100:
        try:
            _, res = c.look("calibrate")
            if res is None:
                continue
            focal_lengths.append(res)
        except EyesException as e:
            print(e)
            continue
        i += 1
        if cv.waitKey(20) == ord('q'):
            break

    focal_length = sum(focal_lengths) / 100
    with open("./calibration_results/focal_length_calibration.txt", 'w') as f:
        f.write("begin f\n")
        f.write(str(focal_length))
        f.write("\n")
        f.write("end f")


def distortion_calibration(c: eyes.Eyes):

    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    chessboard_case_coors = np.zeros((c.chessboard_form[1] * c.chessboard_form[0], 3), np.float32)
    chessboard_case_coors[:, :2] = np.mgrid[0:c.chessboard_form[0], 0:c.chessboard_form[1]].T.reshape(-1, 2)
    real_world_space = []
    image_plane_space = []

    board_shape = None

    while len(image_plane_space) < 10:
        try:
            _, (ret, corners, transformed) = c.look("calibrate")
            board_shape = transformed.shape[::-1]
            if ret == True:
                real_world_space.append(chessboard_case_coors)
                corners2 = cv.cornerSubPix(transformed, corners, c.search_window, c.zero_zone, criteria)
                image_plane_space.append(corners)
        except EyesException as e:
            print(e)
            continue
        if cv.waitKey(1000) == ord('q'):
            break
    _, mtx, dist, _, _ = cv.calibrateCamera(real_world_space, image_plane_space, board_shape, None, None)

    with open("./calibration_results/distortion_calibration.txt", 'w') as f:
        f.write("begin cm\n")
        f.write(str(mtx))
        f.write("\n")
        f.write("end cm\n")
        f.write("begin dc\n")
        f.write(str(dist))
        f.write("\n")
        f.write("end dc")


if __name__ == "__main__":
    c = eyes.Eyes(2, **{
        'known_width': 5,
        'known_distance': 20,
        'color_transform': cv.COLOR_RGB2GRAY,
        'rotation': 180,
        'chessboard_form': (7, 6),
        'search_window': (11, 11),
        'zero_zone': (-1, -1),
        'focal_calibration': False,
        'distortion_calibration': True
    })

    if c.focal_calibration:
        focal_calibration(c)
    elif c.distortion_calibration:
        distortion_calibration(c)

    c.video_flow.release()
    cv.destroyAllWindows()
