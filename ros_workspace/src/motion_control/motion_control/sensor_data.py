import math
from motion_control.const import *

def sign(x):
    if x < 0:
        return -1
    else:
        return 1

class SensorData():
    
    pos_x = 0
    pos_y = 0
    pos_angle_ = 0
    pos_angle = 0
    team = "yellow"
    robot = "slave"

    object_in_game_area = []

    def atomic_move(cls, left_ticks : int, right_ticks: int) -> None:
        dbeta = 0; 
        cos_angle = math.cos(cls.pos_angle_)
        sin_angle = math.sin(cls.pos_angle_)
        if(sign(left_ticks) == sign(right_ticks)):
            c_ticks = sign(left_ticks) * min(abs(left_ticks), abs(right_ticks))
            dl = c_ticks * TICKS_TO_MM
            cls.pos_x += (dl * cos_angle)
            cls.pos_y += (dl * sin_angle)

            if cls.robot == "master":
                dbeta = TICKS_TO_RADIANS_MASTER * (left_ticks - right_ticks)
            elif cls.robot == "slave":
                dbeta = TICKS_TO_RADIANS_SLAVE * (left_ticks - right_ticks)

        elif(left_ticks < 0 and right_ticks >= 0):
            if cls.robot == "master":
                alpha = min(abs(left_ticks), abs(right_ticks)) * TICKS_TO_RADIANS_HALF_BASE_MASTER
                cls.pos_angle_ -= alpha
            elif cls.robot == "slave":
                alpha = min(abs(left_ticks), abs(right_ticks)) * TICKS_TO_RADIANS_HALF_BASE_SLAVE
                cls.pos_angle_ -= alpha

            if cls.robot == "master":
                dbeta = -TICKS_TO_RADIANS_MASTER * abs(abs(left_ticks) - abs(right_ticks))
            elif cls.robot == "slave":
                dbeta = -TICKS_TO_RADIANS_SLAVE * abs(abs(left_ticks) - abs(right_ticks))

        elif(left_ticks >= 0 and right_ticks < 0):

            if cls.robot == "master":
                alpha = min(abs(left_ticks), abs(right_ticks)) * TICKS_TO_RADIANS_HALF_BASE_MASTER
                cls.pos_angle_ += alpha
                dbeta = TICKS_TO_RADIANS_MASTER * abs(abs(left_ticks) - abs(right_ticks)); 
            elif cls.robot == "slave":
                alpha = min(abs(left_ticks), abs(right_ticks)) * TICKS_TO_RADIANS_HALF_BASE_SLAVE
                cls.pos_angle_ += alpha
                dbeta = TICKS_TO_RADIANS_SLAVE * abs(abs(left_ticks) - abs(right_ticks)); 
        
        cls.pos_angle_ += dbeta
        dr = 0
        if cls.robot == "master":
            dr = dbeta * (WHEEL_DISTANCE_MASTER / 2)
        elif cls.robot == "slave":
            dr = dbeta * (WHEEL_DISTANCE_SLAVE / 2)
        cls.pos_x += dr * (1 - dbeta * dbeta / 2)
        cls.pos_y += dr * dbeta
        theta = abs(cls.pos_angle_) - 2 * math.pi * ((int) (abs(cls.pos_angle_) / (2 * math.pi)))
        if cls.pos_angle_ < 0:
            cls.pos_angle = 2 * math.pi - theta
        else:
            cls.pos_angle = theta