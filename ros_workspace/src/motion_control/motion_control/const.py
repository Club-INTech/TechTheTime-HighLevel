import math

WHEEL_DIAMETER_MM = 68.0
TICKS_PER_TURN = 1024
HALF_LENGTH_SLAVE = 75
HALF_WIDTH_SLAVE = 123
RADIUS_BASE_SLAVE = 143.962
HALF_LENGTH_MASTER = 83.9925
HALF_WIDTH_MASTER = 170
RADIUS_BASE_MASTER = 184.817

WHEEL_DISTANCE_SLAVE = 278.0
WHEEL_DISTANCE_MASTER = 331.0
HALF_WHEEL_DISTANCE_SLAVE = WHEEL_DISTANCE_SLAVE / 2
HALF_WHEEL_DISTANCE_MASTER = WHEEL_DISTANCE_MASTER / 2

TICKS_TO_MM = (math.pi * WHEEL_DIAMETER_MM) / (2 * TICKS_PER_TURN)
MM_TO_TICKS =  1 / TICKS_TO_MM

TICKS_TO_RADIANS_SLAVE = TICKS_TO_MM / WHEEL_DISTANCE_SLAVE
RADIANS_TO_TICKS_SLAVE = 1 / TICKS_TO_RADIANS_SLAVE
TICKS_TO_RADIANS_MASTER = TICKS_TO_MM / WHEEL_DISTANCE_MASTER
RADIANS_TO_TICKS_MASTER = 1 / TICKS_TO_RADIANS_MASTER

TICKS_TO_RADIANS_HALF_BASE_SLAVE = 2 * TICKS_TO_RADIANS_SLAVE
RADIANS_TO_TICKS_HALF_BASE_SLAVE = 1 / TICKS_TO_RADIANS_HALF_BASE_SLAVE
TICKS_TO_RADIANS_HALF_BASE_MASTER = 2 * TICKS_TO_RADIANS_MASTER
RADIANS_TO_TICKS_HALF_BASE_MASTER = 1 / TICKS_TO_RADIANS_HALF_BASE_MASTER

START_X_2A_YELLOW = 257.0
START_Y_2A_YELLOW = 915.0
START_ANGLE_2A_YELLOW = math.pi/2

START_X_2A_PURPLE = 2590.0
START_Y_2A_PURPLE = 75.0
START_ANGLE_2A_PURPLE = math.pi

START_X_1A_YELLOW = 570.0
START_Y_1A_YELLOW = 579.0
START_ANGLE_1A_YELLOW = 0

START_X_1A_PURPLE = 2820.0
START_Y_1A_PURPLE = 750
START_ANGLE_1A_PURPLE= math.pi

DEFAULT_SCAN_PROCESSING_DELAY_NS = 5000000
DEFAULT_STOP_PRECISION_MM = 250