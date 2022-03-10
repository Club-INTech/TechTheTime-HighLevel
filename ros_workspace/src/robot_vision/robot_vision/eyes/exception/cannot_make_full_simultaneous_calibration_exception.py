from robot_vision.eyes.exception.eyes_exception import EyesException


class CannotMakeFullSimultaneousCalibrationException(EyesException):
    def __init__(self, expression: str):
        super(CannotMakeFullSimultaneousCalibrationException, self).__init__(expression,
                                                "Cannot make distortion and focal length calibration at the same time")
