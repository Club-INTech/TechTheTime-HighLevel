import math
from motion_control.const import *

def sign(x):
    if x < 0:
        return -1
    else:
        return 1

class SensorData():
    
    pos_x = BEGIN_X_2A
    pos_y = BEGIN_Y_2A
    pos_angle_ = 0
    pos_angle = 0

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
            dbeta = TICKS_TO_RADIANS * (left_ticks - right_ticks)
        elif(left_ticks < 0 and right_ticks >= 0):
            alpha = min(abs(left_ticks), abs(right_ticks)) * TICKS_TO_RADIANS_HALF_BASE
            cls.pos_angle_ -= alpha
            dbeta = -TICKS_TO_RADIANS * abs(abs(left_ticks) - abs(right_ticks)); 
        elif(left_ticks >= 0 and right_ticks < 0):
            alpha = min(abs(left_ticks), abs(right_ticks)) * TICKS_TO_RADIANS_HALF_BASE
            cls.pos_angle_ += alpha
            dbeta = TICKS_TO_RADIANS * abs(abs(left_ticks) - abs(right_ticks)); 
        
        cls.pos_angle_ += dbeta
        dr = dbeta * (WHEEL_DISTANCE / 2)
        cls.pos_x += dr * (1 - dbeta * dbeta / 2)
        cls.pos_y += dr * dbeta
        theta = abs(cls.pos_angle_) - 2 * math.pi * ((int) (abs(cls.pos_angle_) / (2 * math.pi)))
        if cls.pos_angle_ < 0:
            cls.pos_angle = 2 * math.pi - theta
        else:
            cls.pos_angle = theta