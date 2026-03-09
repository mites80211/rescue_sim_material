import math
import os

import numpy as np

# ---- ROBOT
ROBOT_TIME_STEP = 32
ROBOT_DIAMETER = 0.071
ROBOT_RADIUS = ROBOT_DIAMETER / 2.0
ROBOT_AXLE_LENGTH = 0.048
ROBOT_HALF_AXLE_LENGTH = ROBOT_AXLE_LENGTH / 2.0
ROBOT_WHEEL_RADIUS = 0.02
ROBOT_MAX_VELOCITY = 6.28
ROBOT_TURN_VALUE = math.pi / 6
ROBOT_TURN_GAIN = 10
ROBOT_FOLLOW_DISTANCE = 0.02
ROBOT_EXIT_TIME = 30

# ---- RECEIVER
RECEIVER_NAME = 'receiver'

# ---- EMITTER
EMITTER_NAME = 'emitter'

# ---- MOTORS
LEFT_WHEEL_MOTOR_NAME = 'left wheel motor'
RIGHT_WHEEL_MOTOR_NAME = 'right wheel motor'

# ---- INERTIAL UNIT
INERTIAL_UNIT_NAME = 'inertial unit'

# ---- GPS
GPS_NAME = 'global positioning system'

# ---- CAMERA
LEFT_CAMERA_NAME = 'left camera'
RIGHT_CAMERA_NAME = 'right camera'

# ---- COLOR SENSOR
COLOR_SENSOR_NAME = 'color sensor'

# ---- DISTANCE SENSORS
FRONT_HOLE_SENSOR_NAME = 'front dist sensor'
LEFT_HOLE_SENSOR_NAME = 'left dist sensor'
RIGHT_HOLE_SENSOR_NAME = 'right dist sensor'
FRONT_HOLE_OFFSET = np.array((0, 0, -0.032))
LEFT_HOLE_OFFSET = np.array((-0.024, 0, -0.024))
RIGHT_HOLE_OFFSET = np.array((0.024, 0, -0.024))

# ---- LIDAR
LIDAR_NAME = 'lidar'
LIDAR_MAX_RANGE = 0.36

# ---- MAP
MAP_STAR = np.array([(-1, 0), (1, 0), (0, 1), (0, -1)])
MAP_SQUARE = np.array([(-1, 0), (1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)])
MAP_SHAPE = np.array((1998, 1998))
MAP_OFFSET = 1002
MAP_SPACING = 400
MAP_UNKNOWN = 0
MAP_FREE = 1
MAP_DILATION = 2
MAP_HOLE = 3
MAP_WALL = 4

BONUS_MAP_EMPTY = ord('0')
BONUS_MAP_WALL = ord('1')
BONUS_MAP_HOLE = ord('2')
BONUS_MAP_SWAMP = ord('3')
BONUS_MAP_CHECKPOINT = ord('4')
BONUS_MAP_START = ord('5')
BONUS_MAP_BLUE = ord('6')
BONUS_MAP_PURPLE = ord('7')
BONUS_MAP_RED = ord('8')
BONUS_MAP_GREEN = ord('9')

EDT_MIN_DISTANCE = round(0.0325 * MAP_SPACING)
EDT_MAX_DISTANCE = round(0.0500 * MAP_SPACING)

CLS_TO_LETTER = {
    6.: 'S',
    7.: 'U',
    3.: 'H',
    1.: 'D',
    5.: 'P',
    0.: 'C',
    4.: 'O',
    2.: 'F'
}
