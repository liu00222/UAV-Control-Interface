import numpy as np
import pickle

from kivy.core.window import Window

from src.glob import *
from src.util import process_point, make_plot
import src.util as util


class Tracer3:
    def __init__(self, s, num_robot):
        self.num_robot = num_robot

        # Container for the touch points
        self.touch_points = []

        # Recorder for the number of fingers involved in the current gesture
        self.current_finger_num = 0
        self.type = NO_GESTURE

        # Connection to the server
        self.s = s

        # If the UAVs are landing on
        self.land_on = False

        # If we are in the top_down view
        self.top_down_view = False

        # The type of current formation
        # ["none", "triangle", "horizontal", "vertical"]
        self.formation = "none"

        # The distance parameter in each formation
        self.d = 1
        self.v = 1
        self.h = 1

        # Load the predictor (recognizer)
        with open("model/recognizer_1.pkl", "rb") as f:
            self.clf = pickle.load(f)

    def add(self, x, y):
        """
        Add the point (x, y) into the container self.touch_points

        :param x: <float> x coordinate
        :param y: <float> y coordinate
        """
        self.touch_points.append([x, y])

    def touch_down_update(self, x, y):
        """
        Update the tracer when touch down in GESTURE mode

        :param x: <float> x coordinate
        :param y: <float> y coordinate
        """
        if self.current_finger_num == 0:
            self.current_finger_num = 1
            self.type = ONE
            # print("Finger 1 received")
        elif self.current_finger_num == 1:
            self.current_finger_num = 2
            self.type = TWO
            # print("Finger 2 received")
        elif self.current_finger_num == 2:
            self.current_finger_num = 3
            self.type = THREE
            # print("Finger 3 received")
        self.add(x, y)

    def touch_move_update(self, x, y):
        """
        Update the tracer when touch move in GESTURE mode

        :param x: <float> x coordinate
        :param y: <float> y coordinate
        """
        self.add(x, y)

    def touch_up_update(self, x, y):
        """
        Update the tracer when touch up in GESTURE mode

        :param x: <float> x coordinate
        :param y: <float> y coordinate
        """
        self.add(x, y)

        # update the number of fingers currently on the screen
        if self.current_finger_num == 3 or self.current_finger_num == 2 or self.current_finger_num == 1:
            self.current_finger_num = self.current_finger_num - 1

        if self.current_finger_num == 0 and self.top_down_view:
            prediction = self.predict(self.obtain_images())
            # s shape
            if prediction == 0:
                self.s.send("specific speed".encode('utf-8'))
                self.formation = "triangle"
                self.d = 1
                util.move_robots("specific speed", self.d, self.num_robot)

            # check
            elif prediction == 1:
                self.s.send("decrease speed".encode('utf-8'))
                if self.formation == "triangle":
                    self.d = self.d * 0.8
                    util.move_robots("specific speed", self.d, self.num_robot)
                elif self.formation == "vertical":
                    self.v = self.v * 0.8
                    util.move_robots("vertical", self.v, self.num_robot)
                elif self.formation == "horizontal":
                    self.h = self.h * 0.8
                    util.move_robots("horizontal", self.h, self.num_robot)

            # check reverse
            elif prediction == 2:
                self.s.send("increase speed".encode('utf-8'))
                if self.formation == "triangle":
                    self.d = self.d * 1.2
                    util.move_robots("specific speed", self.d, self.num_robot)
                elif self.formation == "vertical":
                    self.v = self.v * 1.2
                    util.move_robots("vertical", self.v, self.num_robot)
                elif self.formation == "horizontal":
                    self.h = self.h * 1.2
                    util.move_robots("horizontal", self.h, self.num_robot)

            # horizontal
            elif prediction == 3:
                self.s.send("horizontal".encode('utf-8'))
                self.formation = "horizontal"
                self.h = 1
                util.move_robots("horizontal", self.h, self.num_robot)

            # vertical
            elif prediction == 4:
                self.s.send("vertical".encode('utf-8'))
                self.formation = "vertical"
                self.v = 1
                util.move_robots("vertical", self.v, self.num_robot)

            # receive data from the server (currently useless, but left for future usage)
            temp_data, temp_addr = self.s.recvfrom(1024)

            # clear the container
            self.touch_points = []

        # if the gesture is finished, recognize the gesture and send data to the server
        if self.current_finger_num == 0 and not self.top_down_view:
            # print("The number of fingers of this gesture is: " + str(self.type))
            # self.report()
            prediction = self.predict(self.obtain_images())
            self.send_request(prediction, self.touch_points[0], self.touch_points[-1])

            # receive data from the server (currently useless, but left for future usage)
            temp_data, temp_addr = self.s.recvfrom(1024)

            # clear the container
            self.touch_points = []

    def report(self):
        """
        Print the coordinates of the points of the current gesture

        """
        print("[", end="", file=open("check.txt", "a"))
        for i in range(len(self.touch_points)):
            p = self.touch_points[i]
            print("(" + str(p[0] / Window.size[0]) + "," + str(p[1] / Window.size[1]) + ")", end="",
                  file=open("check.txt", "a"))
            if i != len(self.touch_points) - 1:
                print(";", end="", file=open("check.txt", "a")),
        print("]", file=open("check.txt", "a"))

    def obtain_images(self):
        """
        Process the current gesture points and return a 2D list containing the normalized points

        :return: <2D list> after normalization
        """
        buffer = ""
        buffer += "["
        for i in range(len(self.touch_points)):
            p = self.touch_points[i]
            buffer += "("
            buffer += str(p[0] / Window.size[0])
            buffer += ","
            buffer += str(p[1] / Window.size[1])
            buffer += ")"
            if i != len(self.touch_points) - 1:
                buffer += ";"
        buffer += "]"
        points = process_point(buffer)
        image = make_plot(points)
        return image

    def predict(self, image):
        """
        Make prediction based on the image

        :param image: <2D list> the current gesture after processing
        :return: <int> result of the prediction
        """
        # flatten the image
        x = ((np.array(image)).reshape(729)).tolist()
        prediction = self.clf.predict([x])

        return prediction[0]

    def send_request(self, prediction, start, end):
        # s shape
        if prediction == 0:
            self.s.send("specific speed".encode('utf-8'))
            self.formation = "triangle"
            self.d = 1
            util.move_robots("specific speed", self.d, self.num_robot)
            return

        # check
        elif prediction == 1:
            self.s.send("decrease speed".encode('utf-8'))
            if self.formation == "triangle":
                self.d = self.d * 0.8
                util.move_robots("specific speed", self.d, self.num_robot)
            elif self.formation == "vertical":
                self.v = self.v * 0.8
                util.move_robots("vertical", self.v, self.num_robot)
            elif self.formation == "horizontal":
                self.h = self.h * 0.8
                util.move_robots("horizontal", self.h, self.num_robot)
            return

        # check reverse
        elif prediction == 2:
            self.s.send("increase speed".encode('utf-8'))
            if self.formation == "triangle":
                self.d = self.d * 1.2
                util.move_robots("specific speed", self.d, self.num_robot)
            elif self.formation == "vertical":
                self.v = self.v * 1.2
                util.move_robots("vertical", self.v, self.num_robot)
            elif self.formation == "horizontal":
                self.h = self.h * 1.2
                util.move_robots("horizontal", self.h, self.num_robot)
            return

        # horizontal
        elif prediction == 3:
            if left_screen(start) and left_screen(end):
                if slide_leftward(start, end):
                    self.s.send("leftward".encode('utf-8'))
                    if not self.land_on:
                        util.move_robots("leftward", 0, self.num_robot)
                    return
                elif slide_rightward(start, end):
                    self.s.send("rightward".encode('utf-8'))
                    if not self.land_on:
                        util.move_robots("rightward", 0, self.num_robot)
                    return
            elif left_screen(start) and left_screen(end):
                if slide_leftward(start, end):
                    self.s.send("left rotation".encode('utf-8'))
                    return
                elif slide_rightward(start, end):
                    self.s.send("right rotation".encode('utf-8'))
                    return

        # vertical
        elif prediction == 4:
            if left_screen(start) and left_screen(end):
                if slide_upward(start, end):
                    self.s.send("forward".encode('utf-8'))
                    if not self.land_on:
                        util.move_robots("forward", 0, self.num_robot)
                    return
                elif slide_downward(start, end):
                    self.s.send("backward".encode('utf-8'))
                    if not self.land_on:
                        util.move_robots("backward", 0, self.num_robot)
                    return
            elif right_screen(start) and right_screen(end):
                if slide_upward(start, end):
                    self.s.send("upward".encode('utf-8'))
                    if not self.land_on:
                        util.move_robots("upward", 0, self.num_robot)
                    return
                elif slide_downward(start, end):
                    self.s.send("downward".encode('utf-8'))
                    if not self.land_on:
                        util.move_robots("downward", 0, self.num_robot)
                    return

        # from left bottom to right top
        elif prediction == 5:
            if left_screen(start) and right_screen(end):
                self.s.send("take off".encode('utf-8'))
                if self.land_on:
                    util.move_robots("take off", 0, self.num_robot)
                    self.land_on = False
                return
            elif left_screen(end) and right_screen(start):
                self.s.send("land on".encode('utf-8'))
                if not self.land_on:
                    util.move_robots("land on", 0, self.num_robot)
                    self.land_on = True
                return

        # from left top to right bottom
        elif prediction == 6:
            if left_screen(start) and right_screen(end):
                self.s.send("hover".encode('utf-8'))
                return
            elif left_screen(end) and right_screen(start):
                self.s.send("maintain".encode('utf-8'))
                return
        self.s.send("nothing".encode('utf-8'))


def slide_upward(start, end):
    return end[1] > start[1]


def slide_downward(start, end):
    return start[1] > end[1]


def left_screen(point):
    return point[0] < Window.size[0] / 2


def right_screen(point):
    return point[0] > Window.size[0] / 2


def slide_leftward(start, end):
    return end[0] < start[0]


def slide_rightward(start, end):
    return start[0] < end[0]
