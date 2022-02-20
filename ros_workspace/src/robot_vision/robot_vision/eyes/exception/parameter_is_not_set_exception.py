from robot_vision.eyes.exception.eyes_exception import EyesException


class ParameterIsNotSetException(EyesException):
    def __init__(self, expression: str):
        super(ParameterIsNotSetException, self).__init__(expression, "Parameter required but not set")
