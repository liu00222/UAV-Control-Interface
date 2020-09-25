from kivy.app import App
import numpy as np

from src.glob import *
from src.glob import coordinate_multiplier
from src.mymath import my_rounding


class Robot:
    def __init__(self, id, x, y, z, length, width, height, type):
        self.id = str(id)
        self.x = x
        self.y = y
        self.z = z
        self.length = length
        self.width = width
        self.height = height
        self.type = type
    
    def get_id(self):
        """
        Return the ID of the robot

        :return: <string> the ID of the robot
        """
        return self.id
        
    def set_position(self, x, y, z):
        """
        Set the position of the robot

        :param x: <float> x coordinate
        :param y: <float> y coordinate
        :param z: <float> z coordinate
        """
        self.x = x
        self.y = y
        self.z = z


def receive_position_1(*dt):
    global s
    temp1, temp2 = s.recvfrom(1024)

    data, addr = s.recvfrom(1024)
    pos_temp = (data.decode('utf-8')).split('&')
    if len(pos_temp) == 4:
        if pos_temp[0] == 't':
            position_t[0] = float(pos_temp[1])*coordinate_multiplier
            position_t[1] = 0.1
            position_t[2] = float(pos_temp[2])*(-coordinate_multiplier)
        else:
            id = int(pos_temp[0])
            if id == 0:
                position_1[0] = float(pos_temp[1])*coordinate_multiplier
                position_1[1] = float(pos_temp[3])*coordinate_multiplier
                position_1[2] = float(pos_temp[2])*(-coordinate_multiplier)
            elif id == 1:
                position_2[0] = float(pos_temp[1])*coordinate_multiplier
                position_2[1] = float(pos_temp[3])*coordinate_multiplier
                position_2[2] = float(pos_temp[2])*(-coordinate_multiplier)
            elif id == 2:
                position_3[0] = float(pos_temp[1])*coordinate_multiplier
                position_3[1] = float(pos_temp[3])*coordinate_multiplier
                position_3[2] = float(pos_temp[2])*(-coordinate_multiplier)
            elif id == 3:
                position_4[0] = float(pos_temp[1])*coordinate_multiplier
                position_4[1] = float(pos_temp[3])*coordinate_multiplier
                position_4[2] = float(pos_temp[2])*(-coordinate_multiplier)
            elif id == 4:
                position_5[0] = float(pos_temp[1])*coordinate_multiplier
                position_5[1] = float(pos_temp[3])*coordinate_multiplier
                position_5[2] = float(pos_temp[2])*(-coordinate_multiplier)

    data, addr = s.recvfrom(1024)
    pos_temp = (data.decode('utf-8')).split('&')
    if len(pos_temp) == 4:
        if pos_temp[0] == 't':
            position_t[0] = float(pos_temp[1])*coordinate_multiplier
            position_t[1] = 0.1
            position_t[2] = float(pos_temp[2])*(-coordinate_multiplier)
        else:
            id = int(pos_temp[0])
            if id == 0:
                position_1[0] = float(pos_temp[1])*coordinate_multiplier
                position_1[1] = float(pos_temp[3])*coordinate_multiplier
                position_1[2] = float(pos_temp[2])*(-coordinate_multiplier)
            elif id == 1:
                position_2[0] = float(pos_temp[1])*coordinate_multiplier
                position_2[1] = float(pos_temp[3])*coordinate_multiplier
                position_2[2] = float(pos_temp[2])*(-coordinate_multiplier)
            elif id == 2:
                position_3[0] = float(pos_temp[1])*coordinate_multiplier
                position_3[1] = float(pos_temp[3])*coordinate_multiplier
                position_3[2] = float(pos_temp[2])*(-coordinate_multiplier)
            elif id == 3:
                position_4[0] = float(pos_temp[1])*coordinate_multiplier
                position_4[1] = float(pos_temp[3])*coordinate_multiplier
                position_4[2] = float(pos_temp[2])*(-coordinate_multiplier)
            elif id == 4:
                position_5[0] = float(pos_temp[1])*coordinate_multiplier
                position_5[1] = float(pos_temp[3])*coordinate_multiplier
                position_5[2] = float(pos_temp[2])*(-coordinate_multiplier)

    data, addr = s.recvfrom(1024)
    pos_temp = (data.decode('utf-8')).split('&')
    if len(pos_temp) == 4:
        if pos_temp[0] == 't':
            position_t[0] = float(pos_temp[1])*coordinate_multiplier
            position_t[1] = 0.1
            position_t[2] = float(pos_temp[2])*(-coordinate_multiplier)
        else:
            id = int(pos_temp[0])
            if id == 0:
                position_1[0] = float(pos_temp[1])*coordinate_multiplier
                position_1[1] = float(pos_temp[3])*coordinate_multiplier
                position_1[2] = float(pos_temp[2])*(-coordinate_multiplier)
            elif id == 1:
                position_2[0] = float(pos_temp[1])*coordinate_multiplier
                position_2[1] = float(pos_temp[3])*coordinate_multiplier
                position_2[2] = float(pos_temp[2])*(-coordinate_multiplier)
            elif id == 2:
                position_3[0] = float(pos_temp[1])*coordinate_multiplier
                position_3[1] = float(pos_temp[3])*coordinate_multiplier
                position_3[2] = float(pos_temp[2])*(-coordinate_multiplier)
            elif id == 3:
                position_4[0] = float(pos_temp[1])*coordinate_multiplier
                position_4[1] = float(pos_temp[3])*coordinate_multiplier
                position_4[2] = float(pos_temp[2])*(-coordinate_multiplier)
            elif id == 4:
                position_5[0] = float(pos_temp[1])*coordinate_multiplier
                position_5[1] = float(pos_temp[3])*coordinate_multiplier
                position_5[2] = float(pos_temp[2])*(-coordinate_multiplier)


def update_robot_1_x(*dt):
    app = App.get_running_app()
    for j in range(OBJ_LENGTH):
        app.cube[0 * OBJ_LENGTH + j].pos.x = position_1[0]


def update_robot_1_y(*dt):
    app = App.get_running_app()
    for j in range(OBJ_LENGTH):
        app.cube[0 * OBJ_LENGTH + j].pos.y = position_1[1]


def update_robot_1_z(*dt):
    app = App.get_running_app()
    for j in range(OBJ_LENGTH):
        app.cube[0 * OBJ_LENGTH + j].pos.z = position_1[2]


def update_robot_2_x(*dt):
    app = App.get_running_app()
    for j in range(OBJ_LENGTH):
        app.cube[1 * OBJ_LENGTH + j].pos.x = position_2[0]


def update_robot_2_y(*dt):
    app = App.get_running_app()
    for j in range(OBJ_LENGTH):
        app.cube[1 * OBJ_LENGTH + j].pos.y = position_2[1]


def update_robot_2_z(*dt):
    app = App.get_running_app()
    for j in range(OBJ_LENGTH):
        app.cube[1 * OBJ_LENGTH + j].pos.z = position_2[2]


def update_robot_3_x(*dt):
    app = App.get_running_app()
    for j in range(OBJ_LENGTH):
        app.cube[2 * OBJ_LENGTH + j].pos.x = position_3[0]


def update_robot_3_y(*dt):
    app = App.get_running_app()
    for j in range(OBJ_LENGTH):
        app.cube[2 * OBJ_LENGTH + j].pos.y = position_3[1]


def update_robot_3_z(*dt):
    app = App.get_running_app()
    for j in range(OBJ_LENGTH):
        app.cube[2 * OBJ_LENGTH + j].pos.z = position_3[2]


def update_robot_4_x(*dt):
    app = App.get_running_app()
    for j in range(OBJ_LENGTH):
        app.cube[3 * OBJ_LENGTH + j].pos.x = position_4[0]


def update_robot_4_y(*dt):
    app = App.get_running_app()
    for j in range(OBJ_LENGTH):
        app.cube[3 * OBJ_LENGTH + j].pos.y = position_4[1]


def update_robot_4_z(*dt):
    app = App.get_running_app()
    for j in range(OBJ_LENGTH):
        app.cube[3 * OBJ_LENGTH + j].pos.z = position_4[2]


def update_robot_5_x(*dt):
    app = App.get_running_app()
    for j in range(OBJ_LENGTH):
        app.cube[4 * OBJ_LENGTH + j].pos.x = position_5[0]


def update_robot_5_y(*dt):
    app = App.get_running_app()
    for j in range(OBJ_LENGTH):
        app.cube[4 * OBJ_LENGTH + j].pos.y = position_5[1]


def update_robot_5_z(*dt):
    app = App.get_running_app()
    for j in range(OBJ_LENGTH):
        app.cube[4 * OBJ_LENGTH + j].pos.z = position_5[2]

def update_robot_t_x(*dt):
    app = App.get_running_app()
    global robot_num
    for j in range(TARGET_OBJ_LENGTH):
        app.cube[(robot_num - 1) * TARGET_OBJ_LENGTH + j].pos.x = position_t[0]


def update_robot_t_y(*dt):
    app = App.get_running_app()
    global robot_num
    for j in range(TARGET_OBJ_LENGTH):
        app.cube[(robot_num - 1) * TARGET_OBJ_LENGTH + j].pos.y = position_t[1]


def update_robot_t_z(*dt):
    app = App.get_running_app()
    global robot_num
    for j in range(TARGET_OBJ_LENGTH):
        app.cube[(robot_num - 1) * TARGET_OBJ_LENGTH + j].pos.z = position_t[2]
