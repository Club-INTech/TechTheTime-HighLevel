from robot_vision.eyes.exception.eyes_exception import EyesException


class CannotOpenCameraException(EyesException):
    def __init__(self, expression: str):
        super(CannotOpenCameraException, self).__init__(expression, "Cannot open camera. Try different capture id")
