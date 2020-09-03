from kivy.app import App
import numpy as np

from src.glob import *
from src.mymath import my_rounding


def adjust_aspect(*args):
    app = App.get_running_app()
    rsize = app.renderer.size
    aspect = rsize[0] / float(rsize[1])
    app.renderer.camera.aspect = aspect


def _update_obj(dt):
    obj = App.get_running_app().sphere
    if obj.pos.z > -10:
        obj.pos.z -= 0.1


def kivy_matrix_to_np_matrix(mat, num_row, num_col):
    """
    Convert the kivy matrix to a standard numpy matrix

    :param mat: <kivy.graphics.transformation.matrix>
    :param num_col: <int> the number of columns in the matrix
    :param num_row: <int> the number of rows in the matrix

    :return: <np.array>
    """
    lst = mat.get()
    matrix = []
    for i in range(num_row):
        matrix.append([])
        for j in range(num_col):
            matrix[i].append(lst[i * num_row + j])
    return np.array(matrix)


def drop(text, char_lst):
    """
    Drop all the characters that are specified by char_list in text

    :param text: <string> the target string
    :param char_lst: <char list> a list of characters to be dropped
    :return: <string>
    """
    buffer = ""
    for i in range(len(text)):
        if text[i] not in char_lst:
            buffer += text[i]
    return buffer


def process_point(text):
    """
    Transfer a string of points to a list of points

    :param text: <string> a string of points in a specific rule
    :return: <2D list> contains the points that are passed by the gesture
    """
    text = drop(text, ['[', ']', '(', ')', '\n'])
    raw = text.split(';')
    points = []
    for i in range(len(raw)):
        point = raw[i].split(',')
        points.append(
            [my_rounding(float(point[0]) * float(NUM_PIXEL)), my_rounding(float(point[1]) * float(NUM_PIXEL))])
    return points


def make_plot(shape):
    """
    Take a 2D list which contains the points that are passed by the gesture and return the whole image points

    :param shape: <2D list> contains the points that are passed by the gesture
    :return: <2D list> the whole image
    """
    ex = []
    for i in range(NUM_PIXEL):
        ex.append([-1.0] * NUM_PIXEL)

    x0 = shape[0][1]
    y0 = shape[0][0]
    for point in shape:
        if x0 > point[1]:
            x0 = point[1]
        if y0 > point[0]:
            y0 = point[0]

    for j in range(len(shape)):
        shape[j][1] -= x0
        shape[j][0] -= y0

    for point in shape:
        ex[NUM_PIXEL - point[1] - 1][point[0]] += 1.0
    return ex


def stabilize_view(camera_pose_x, camera_pose_y, camera_pose_z):
    app = App.get_running_app()
    app.camera.pos.x = camera_pose_x
    app.camera.pos.y = camera_pose_y
    app.camera.pos.z = camera_pose_z
    app.camera.look_at(ORIGIN)


def move_robots(cmd, distance, num_robot):
    app = App.get_running_app()
    if cmd == "leftward":
        if num_robot >= 1:
            position_1[2] -= 1
        if num_robot >= 2:
            position_2[2] -= 1
        if num_robot >= 3:
            position_3[2] -= 1
        if num_robot >= 4:
            position_4[2] -= 1
        if num_robot >= 5:
            position_5[2] -= 1

    elif cmd == "rightward":
        if num_robot >= 1:
            position_1[2] += 1
        if num_robot >= 2:
            position_2[2] += 1
        if num_robot >= 3:
            position_3[2] += 1
        if num_robot >= 4:
            position_4[2] += 1
        if num_robot >= 5:
            position_5[2] += 1

    elif cmd == "forward":
        if num_robot >= 1:
            position_1[0] += 1
        if num_robot >= 2:
            position_2[0] += 1
        if num_robot >= 3:
            position_3[0] += 1
        if num_robot >= 4:
            position_4[0] += 1
        if num_robot >= 5:
            position_5[0] += 1

    elif cmd == "backward":
        if num_robot >= 1:
            position_1[0] -= 1
        if num_robot >= 2:
            position_2[0] -= 1
        if num_robot >= 3:
            position_3[0] -= 1
        if num_robot >= 4:
            position_4[0] -= 1
        if num_robot >= 5:
            position_5[0] -= 1

    elif cmd == "upward":
        if num_robot >= 1:
            position_1[1] += 1
        if num_robot >= 2:
            position_2[1] += 1
        if num_robot >= 3:
            position_3[1] += 1
        if num_robot >= 4:
            position_4[1] += 1
        if num_robot >= 5:
            position_5[1] += 1

    elif cmd == "downward":
        if num_robot >= 1:
            position_1[1] -= 1
        if num_robot >= 2:
            position_2[1] -= 1
        if num_robot >= 3:
            position_3[1] -= 1
        if num_robot >= 4:
            position_4[1] -= 1
        if num_robot >= 5:
            position_5[1] -= 1

    elif cmd == "land on":
        if num_robot >= 1:
            position_1[1] = 0
        if num_robot >= 2:
            position_2[1] = 0
        if num_robot >= 3:
            position_3[1] = 0
        if num_robot >= 4:
            position_4[1] = 0
        if num_robot >= 5:
            position_5[1] = 0

    elif cmd == "take off":
        if num_robot >= 1:
            position_1[1] = 2
        if num_robot >= 2:
            position_2[1] = 2
        if num_robot >= 3:
            position_3[1] = 2
        if num_robot >= 4:
            position_4[1] = 2
        if num_robot >= 5:
            position_5[1] = 2

    elif cmd == "specific speed":
        if num_robot >= 1:
            position_1[0] = 0
            position_1[2] = 2 * distance
            position_1[1] = 2
        if num_robot >= 2:
            position_2[0] = 2 * distance
            position_2[2] = 0
            position_2[1] = 2
        if num_robot >= 3:
            position_3[0] = 0
            position_3[2] = -2 * distance
            position_3[1] = 2
        if num_robot >= 4:
            position_4[0] = -4 * distance
            position_4[2] = 4 * distance
            position_4[1] = 2
        if num_robot >= 5:
            position_5[0] = -4 * distance
            position_5[2] = -4 * distance
            position_5[1] = 2

    elif cmd == "horizontal":
        if num_robot >= 1:
            position_1[0] = 0
            position_1[2] = 2 * distance
            position_1[1] = 2
        if num_robot >= 2:
            position_2[0] = 0
            position_2[2] = 0
            position_2[1] = 2
        if num_robot >= 3:
            position_3[0] = 0
            position_3[2] = -2 * distance
            position_3[1] = 2
        if num_robot >= 4:
            position_4[0] = 0
            position_4[2] = 4 * distance
            position_4[1] = 2
        if num_robot >= 5:
            position_5[0] = 0
            position_5[2] = -4 * distance
            position_5[1] = 2

    elif cmd == "vertical":
        if num_robot >= 1:
            position_1[0] = 2 * distance
            position_1[2] = 0
            position_1[1] = 2
        if num_robot >= 2:
            position_2[0] = 0
            position_2[2] = 0
            position_2[1] = 2
        if num_robot >= 3:
            position_3[0] = -2 * distance
            position_3[2] = 0
            position_3[1] = 2
        if num_robot >= 4:
            position_4[0] = 4 * distance
            position_4[2] = 0
            position_4[1] = 2
        if num_robot >= 5:
            position_5[0] = -4 * distance
            position_5[2] = 0
            position_5[1] = 2
