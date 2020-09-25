from packages.kivy3.geometries import BoxGeometry
from packages.kivy3.materials import Material
from packages.kivy3.mesh import Mesh

import socket

# define macros

# mathematical origin in 3D environment
ORIGIN = [0, 0, 0]

# interface modes
SELECTION_MODE = 0
GESTURE_MODE = 1
VIEW_MODE = 2

# mesh only used for initialization
INITIAL_CUBE = Mesh(geometry=BoxGeometry(0, 0, 0),
                    material=Material(
                        transparency=0,
                        color=(0, 0, 0),
                        specular=(0, 0, 0)
                    ))

# colors used to paint the robots
LIGHT_YELLOW = (1.0, 1.0, .2)
WHITE = (1.0, 1.0, 1.0)
RED = (.9, .1, .1)
OBSTACLE_COLOR = (0.8, 0.8, 0.8)

# types of robots
UAV = 0
UGV = 1

# macros for gesture recognition
NO_GESTURE = 0
ONE = 1
TWO = 2
THREE = 3
NUM_PIXEL = 27

# global socket connection
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# robots centralized properties
coordinate_multiplier = 2.7

position_1 = [0, coordinate_multiplier, coordinate_multiplier]
position_2 = [0, coordinate_multiplier, 0]
position_3 = [0, coordinate_multiplier, -coordinate_multiplier]
position_4 = [0, coordinate_multiplier, 2*coordinate_multiplier]
position_5 = [0, coordinate_multiplier, -2*coordinate_multiplier]
position_t = [4, 0.1, -2*coordinate_multiplier]

OBJ_LENGTH = 14
TARGET_OBJ_LENGTH = 1

robot_num = 0

speed = 0.06
target_speed = 0.1

tolerance = 0.01
