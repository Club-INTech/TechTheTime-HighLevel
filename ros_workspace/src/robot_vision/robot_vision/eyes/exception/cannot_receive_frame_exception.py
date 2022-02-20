from robot_vision.eyes.exception.eyes_exception import EyesException


class CannotReceiveFrameException(EyesException):
    def __init__(self, expression: str):
        super(CannotReceiveFrameException, self).__init__(expression, "Cannot receive frame")
