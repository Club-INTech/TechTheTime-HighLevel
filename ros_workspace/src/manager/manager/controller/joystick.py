import pygame
import sys
from exception.JoystickNotFoundException import JoystickNotFoundException

A = 0
B = 1
X = 2
Y = 3
LEFT_BUMP = 4
RIGHT_BUMP = 5
BACK = 6
START = 7
# GUIDE = 8
LEFT_STICK_BTN = 9
RIGHT_STICK_BTN = 10

# axes
LEFT_STICK_X = 0
LEFT_STICK_Y = 1
RIGHT_STICK_X = 3
RIGHT_STICK_Y = 4
LEFT_TRIGGER = 2
RIGHT_TRIGGER = 5


class Joystick():
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() < 1:
            raise JoystickNotFoundException('pygame.joystick.get_count() < 1', "No joystick was found on the system by pygame.")
        self.mainJoystick = pygame.joystick.Joystick(0)
        self.mainJoystick.init()
    
    def run(self, frequency:int = 0):
        self.run = True
        while self.run:
            for event in pygame.event.get(): # User did something.
                self.doJoystick(event)

    def stop(self):
        self.run = False

    def a(self):
        #print(self.mainJoystick.get_numaxes())
         while True:
            print(self.mainJoystick.get_button(0))
            # if self.mainJoystick.get_button(A):
            #     print('aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')

    def doJoystick(self, event):
        if event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
        elif event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")

    def get_buttons(self):
        return (self.mainJoystick.get_button(A),
        self.mainJoystick.get_button(B),
        self.mainJoystick.get_button(X),
        self.mainJoystick.get_button(Y),
        self.mainJoystick.get_button(LEFT_BUMP),
        self.mainJoystick.get_button(RIGHT_BUMP),
        self.mainJoystick.get_button(BACK),
        self.mainJoystick.get_button(START),
        0, # Unused, since Guide only works on Linux
        self.mainJoystick.get_button(LEFT_STICK_BTN),
        self.mainJoystick.get_button(RIGHT_STICK_BTN))