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