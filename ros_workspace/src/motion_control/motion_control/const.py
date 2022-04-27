import math

WHEEL_DIAMETER_MM = 68.0
TICKS_PER_TURN = 1024
HALF_LENGTH_SLAVE = 75
HALF_WIDTH_SLAVE = 123
RADIUS_BASE_SLAVE = 143.962
HALF_LENGTH_MASTER = 83.9925
HALF_WIDTH_MASTER = 170
RADIUS_BASE_MASTER = 184.817

WHEEL_DISTANCE = 278.0
HALF_WHEEL_DISTANCE = WHEEL_DISTANCE / 2

TICKS_TO_MM = (math.pi * WHEEL_DIAMETER_MM) / (2 * TICKS_PER_TURN)
MM_TO_TICKS =  1 / TICKS_TO_MM

TICKS_TO_RADIANS = TICKS_TO_MM / WHEEL_DISTANCE
RADIANS_TO_TICKS = 1 / TICKS_TO_RADIANS

TICKS_TO_RADIANS_HALF_BASE = 2 * TICKS_TO_RADIANS
RADIANS_TO_TICKS_HALF_BASE = 1 / TICKS_TO_RADIANS_HALF_BASE

BEGIN_X_2A = 250.0
BEGIN_Y_2A = 850.0
BEGIN_ANGLE_2A = 0