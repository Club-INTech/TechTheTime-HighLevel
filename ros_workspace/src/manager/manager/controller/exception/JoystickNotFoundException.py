#This exception is raised when no joystick was found on the system
from email import message


class JoystickNotFoundException(Exception):
    """
    Attributes:
        expression -- input expression in which the error occurred
        message -- explanation of the error
    """

    def __init__(self):
        super.__init__(expression='pygame.joystick.get_count() < 1', message='No joystick was found on the system by pygame')