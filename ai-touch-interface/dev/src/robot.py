from kivy.app import App
import numpy as np

from src.glob import *
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


def update_robot_1_x(*dt):
    app = App.get_running_app()
    if app.cube[0*14].pos.x < position_1[0]:
        for j in range(OBJ_LENGTH):
            app.cube[0*14+j].pos.x += speed_1
    elif app.cube[0*14].pos.x > position_1[0]:
        for j in range(OBJ_LENGTH):
            app.cube[0*14+j].pos.x -= speed_1


def update_robot_1_y(*dt):
    app = App.get_running_app()
    if app.cube[0*14].pos.y < position_1[1]:
        for j in range(OBJ_LENGTH):
            app.cube[0*14+j].pos.y += speed_1
    elif app.cube[0*14].pos.y > position_1[1]:
        for j in range(OBJ_LENGTH):
            app.cube[0*14+j].pos.y -= speed_1


def update_robot_1_z(*dt):
    app = App.get_running_app()
    if app.cube[0*14].pos.z < position_1[2]:
        for j in range(OBJ_LENGTH):
            app.cube[0*14+j].pos.z += speed_1
    elif app.cube[0*14].pos.z > position_1[2]:
        for j in range(OBJ_LENGTH):
            app.cube[0*14+j].pos.z -= speed_1


def update_robot_2_x(*dt):
    app = App.get_running_app()
    if app.cube[1*14].pos.x < position_2[0]:
        for j in range(OBJ_LENGTH):
            app.cube[1*14+j].pos.x += speed_2
    elif app.cube[1*14].pos.x > position_2[0]:
        for j in range(OBJ_LENGTH):
            app.cube[1*14+j].pos.x -= speed_2


def update_robot_2_y(*dt):
    app = App.get_running_app()
    if app.cube[1*14].pos.y < position_2[1]:
        for j in range(OBJ_LENGTH):
            app.cube[1*14+j].pos.y += speed_2
    elif app.cube[1*14].pos.y > position_2[1]:
        for j in range(OBJ_LENGTH):
            app.cube[1*14+j].pos.y -= speed_2


def update_robot_2_z(*dt):
    app = App.get_running_app()
    if app.cube[1*14].pos.z < position_2[2]:
        for j in range(OBJ_LENGTH):
            app.cube[1*14+j].pos.z += speed_2
    elif app.cube[1*14].pos.z > position_2[2]:
        for j in range(OBJ_LENGTH):
            app.cube[1*14+j].pos.z -= speed_2


def update_robot_3_x(*dt):
    app = App.get_running_app()
    if app.cube[2*14].pos.x < position_3[0]:
        for j in range(OBJ_LENGTH):
            app.cube[2*14+j].pos.x += speed_3
    elif app.cube[2*14].pos.x > position_3[0]:
        for j in range(OBJ_LENGTH):
            app.cube[2*14+j].pos.x -= speed_3


def update_robot_3_y(*dt):
    app = App.get_running_app()
    if app.cube[2*14].pos.y < position_3[1]:
        for j in range(OBJ_LENGTH):
            app.cube[2*14+j].pos.y += speed_3
    elif app.cube[2*14].pos.y > position_3[1]:
        for j in range(OBJ_LENGTH):
            app.cube[2*14+j].pos.y -= speed_3


def update_robot_3_z(*dt):
    app = App.get_running_app()
    if app.cube[2*14].pos.z < position_3[2]:
        for j in range(OBJ_LENGTH):
            app.cube[2*14+j].pos.z += speed_3
    elif app.cube[2*14].pos.z > position_3[2]:
        for j in range(OBJ_LENGTH):
            app.cube[2*14+j].pos.z -= speed_3


def update_robot_4_x(*dt):
    app = App.get_running_app()
    if app.cube[3*14].pos.x < position_4[0]:
        for j in range(OBJ_LENGTH):
            app.cube[3*14+j].pos.x += speed_4
    elif app.cube[3*14].pos.x > position_4[0]:
        for j in range(OBJ_LENGTH):
            app.cube[3*14+j].pos.x -= speed_4


def update_robot_4_y(*dt):
    app = App.get_running_app()
    if app.cube[3*14].pos.y < position_4[1]:
        for j in range(OBJ_LENGTH):
            app.cube[3*14+j].pos.y += speed_4
    elif app.cube[3*14].pos.y > position_4[1]:
        for j in range(OBJ_LENGTH):
            app.cube[3*14+j].pos.y -= speed_4


def update_robot_4_z(*dt):
    app = App.get_running_app()
    if app.cube[3*14].pos.z < position_4[2]:
        for j in range(OBJ_LENGTH):
            app.cube[3*14+j].pos.z += speed_4
    elif app.cube[3*14].pos.z > position_4[2]:
        for j in range(OBJ_LENGTH):
            app.cube[3*14+j].pos.z -= speed_4


def update_robot_5_x(*dt):
    app = App.get_running_app()
    if app.cube[4*14].pos.x < position_5[0]:
        for j in range(OBJ_LENGTH):
            app.cube[4*14+j].pos.x += speed_5
    elif app.cube[4*14].pos.x > position_5[0]:
        for j in range(OBJ_LENGTH):
            app.cube[4*14+j].pos.x -= speed_5


def update_robot_5_y(*dt):
    app = App.get_running_app()
    if app.cube[4*14].pos.y < position_5[1]:
        for j in range(OBJ_LENGTH):
            app.cube[4*14+j].pos.y += speed_5
    elif app.cube[4*14].pos.y > position_5[1]:
        for j in range(OBJ_LENGTH):
            app.cube[4*14+j].pos.y -= speed_5


def update_robot_5_z(*dt):
    app = App.get_running_app()
    if app.cube[4*14].pos.z < position_5[2]:
        for j in range(OBJ_LENGTH):
            app.cube[4*14+j].pos.z += speed_5
    elif app.cube[4*14].pos.z > position_5[2]:
        for j in range(OBJ_LENGTH):
            app.cube[4*14+j].pos.z -= speed_5
