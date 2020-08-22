from kivy.app import App
import numpy as np

from src.glob import *
from src.mymath import my_rounding


def rotate_cube(*dt):
    s.send("none".encode('utf-8'))


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


def move_robots(cmd, distance):
    app = App.get_running_app()
    if cmd == "leftward":
        for i in range(len(app.cube)):
            app.cube[i].pos.y += 1.0

    elif cmd == "rightward":
        for i in range(len(app.cube)):
            app.cube[i].pos.y -= 1.0

    elif cmd == "forward":
        for i in range(len(app.cube)):
            app.cube[i].pos.x += 1.0

    elif cmd == "backward":
        for i in range(len(app.cube)):
            app.cube[i].pos.x -= 1.0

    elif cmd == "upward":
        for i in range(len(app.cube)):
            app.cube[i].pos.z += 1.0

    elif cmd == "downward":
        for i in range(len(app.cube)):
            if app.cube[i].pos.z > 0:
                app.cube[i].pos.z -= 1.0

    elif cmd == "land on":
        for i in range(len(app.cube)):
            app.cube[i].pos.z = 0.0

    elif cmd == "take off":
        for i in range(len(app.cube)):
            app.cube[i].pos.z = 1.0

    elif cmd == "specific speed":
        app.cube[0].pos.x = 0
        app.cube[0].pos.y = -1 * distance
        app.cube[0].pos.z = 1

        app.cube[1].pos.x = distance
        app.cube[1].pos.y = 0
        app.cube[1].pos.z = 1

        app.cube[2].pos.x = 0
        app.cube[2].pos.y = distance
        app.cube[2].pos.z = 1

        app.cube[3].pos.x = -2 * distance
        app.cube[3].pos.y = -2 * distance
        app.cube[3].pos.z = 1

        app.cube[4].pos.x = -2 * distance
        app.cube[4].pos.y = 2 * distance
        app.cube[4].pos.z = 1

    elif cmd == "horizontal":
        app.cube[0].pos.x = 0
        app.cube[0].pos.y = -1 * distance
        app.cube[0].pos.z = 1

        app.cube[1].pos.x = 0
        app.cube[1].pos.y = 0
        app.cube[1].pos.z = 1

        app.cube[2].pos.x = 0
        app.cube[2].pos.y = distance
        app.cube[2].pos.z = 1

        app.cube[3].pos.x = 0
        app.cube[3].pos.y = -2 * distance
        app.cube[3].pos.z = 1

        app.cube[4].pos.x = 0
        app.cube[4].pos.y = 2 * distance
        app.cube[4].pos.z = 1

    elif cmd == "vertical":
        app.cube[0].pos.x = -1 * distance
        app.cube[0].pos.y = 0
        app.cube[0].pos.z = 1

        app.cube[1].pos.x = 0
        app.cube[1].pos.y = 0
        app.cube[1].pos.z = 1

        app.cube[2].pos.x = distance
        app.cube[2].pos.y = 0
        app.cube[2].pos.z = 1

        app.cube[3].pos.x = -2 * distance
        app.cube[3].pos.y = 0
        app.cube[3].pos.z = 1

        app.cube[4].pos.x = 2 * distance
        app.cube[4].pos.y = 0
        app.cube[4].pos.z = 1
