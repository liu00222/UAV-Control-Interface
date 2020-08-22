from os.path import join, dirname, abspath
from kivy.core.window import Window
import numpy as np
#import pickle
#import socket

from src.glob import *
from src.util import process_point, make_plot


FOLDER = dirname(abspath(__file__))

class Tracer3:
    def __init__(self):
        self.touch_points = []
        self.current_finger_num = 0
        self.type = NO_GESTURE

        #self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #self.s.connect(('10.100.130.54', 4000))

        #with open("model/model.pkl", "rb") as f:
        #    self.clf = pickle.load(f)

    def add(self, x, y):
        self.touch_points.append([x, y])
    
    def touch_down_update(self, x, y):
        if self.current_finger_num == 0:
            self.current_finger_num = 1
            self.type = ONE
            #print("Finger 1 received")
        elif self.current_finger_num == 1:
            self.current_finger_num = 2
            self.type = TWO
            #print("Finger 2 received")
        elif self.current_finger_num == 2:
            self.current_finger_num = 3
            self.type = THREE
            #print("Finger 3 received")
        self.add(x, y)
    
    def touch_move_update(self, x, y):
        self.add(x, y)
    
    def touch_up_update(self, x, y):
        if self.current_finger_num == 3 or self.current_finger_num == 2 or self.current_finger_num == 1:
            self.current_finger_num = self.current_finger_num - 1
        if self.current_finger_num == 0:
            # print("The number of fingers of this gesture is: " + str(self.type))
            self.report()
            #image = self.obtain_images()
            #prediction = self.predict(image)
            #if prediction == 0:
                # print("set a specific speed")
            #    self.s.send("set a specific speed".encode('utf-8'))
            #    data, addr = self.s.recvfrom(1024)
            #elif prediction == 1:
                # print("decrease the speed")
            #    self.s.send("decrease speed".encode('utf-8'))
            #    data, addr = self.s.recvfrom(1024)
            #elif prediction == 2:
                # print("increase the speed")
            #    self.s.send("increase speed".encode('utf-8'))
            #    data, addr = self.s.recvfrom(1024)
            self.touch_points = []
        self.add(x, y)

    def report(self):
        filename = "lttorb.txt"
        
        print("[", end="", file=open(filename, "a"))
        for i in range(len(self.touch_points)):
            p = self.touch_points[i]
            print("(" + str(p[0] / Window.size[0]) + "," + str(p[1] / Window.size[1]) + ")", end="", file=open(filename, "a"))
            if i != len(self.touch_points) - 1:
                print(";", end="", file=open(filename, "a")),
        print("]", file=open(filename, "a"))

    def obtain_images(self):
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
        # flatten the image
        x = ((np.array(image)).reshape(729)).tolist()
        y_pred = self.clf.predict([x])
        return y_pred[0]