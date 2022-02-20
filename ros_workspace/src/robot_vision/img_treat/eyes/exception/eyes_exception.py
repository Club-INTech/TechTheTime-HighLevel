import time


class EyesException(Exception):
    def __init__(self, expression: str, msg: str):
        super(EyesException, self).__init__(expression, msg)
        time.sleep(3)
