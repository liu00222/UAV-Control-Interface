# CSCI 5521 - Homework 2
# Yupei Liu
# liu00222@umn.edu
# 5415196

# Remove the future warnings from the screen
def warn(*args, **kwargs):
    pass
import warnings
warnings.warn = warn


# Import the necessary libraries for standard operations
import numpy as np
from numpy.linalg import det
from numpy.linalg import inv
import numpy.linalg as linalg
import random

# Import datasets
from sklearn.datasets import load_digits
from sklearn.datasets import load_boston

# Import the fitting method
from sklearn.linear_model import LogisticRegression

import pickle

from sklearn.ensemble import RandomForestClassifier
from sklearn.datasets import load_iris
from pure_sklearn.map import convert_estimator
import matplotlib.pyplot as plt
import torch


NUM_PIXEL = 28


def my_round(num):
    if num - int(num) >= 0.5:
        return int(num + 1)
    return int(num)


def drop(text, char_lst):
    buffer = ""
    for i in range(len(text)):
        if text[i] not in char_lst:
            buffer += text[i]
    return buffer


def process_point(text):
    text = drop(text, ['[', ']', '(', ')', '\n'])
    raw = text.split(';')
    points = []
    for i in range(len(raw)):
        point = raw[i].split(',')
        points.append([my_round(float(point[0]) * float(NUM_PIXEL)), my_round(float(point[1]) * float(NUM_PIXEL))])
    return points


def make_plot(shape):
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



class MyLogisticReg2:

    # constructor

    def __init__(self, d):

        # set the step size as a small constant term
        self.step_size = 1e-6
        self.d = d

        # initialize the weights accoding the textbook contents
        # w0 = w[0] and w = w[1...d]
        self.w = [0]*(d + 1)
        for j in range(d + 1):
            self.w[j] = random.uniform(-0.01, 0.01)


    # the sigmoid function following the definition in the textbook
    def sigmoid(self, o):
        return (1 / (1 + np.exp(-o)))


    # fit the model and store results into w
    def fit(self, X, y):

        N = len(X)
        d = self.d
        convergence = False
        previous_w = self.w[:]

        while not convergence:

            # initialize the gradients of the weights
            # w_gradient[0] is for w0, and w_gradient[1...d] are for w
            w_gradient = []
            for j in range(d + 1):
                w_gradient.append(0)

            # obtain the gradients of the weights
            for t in range(N):
                o = self.w[0] * 1
                for j in range(d):
                    o += self.w[j + 1] * X[t][j]

                y_temp = self.sigmoid(o)

                w_gradient[0] += (y[t] - y_temp)
                for j in range(d):
                    w_gradient[j + 1] += (y[t] - y_temp) * X[t][j]

            # update the weights based on the gradients and the step size
            for j in range(d + 1):
                self.w[j] += self.step_size * w_gradient[j]

            # As Professor said, the criterion to determine the convergence:
            # When the norm of the difference between the current weights
            # and the previous weigths is less than a small number, the
            # convergence happens.

            diff = np.array(self.w) - np.array(previous_w)
            print(np.linalg.norm(diff))
            if np.linalg.norm(diff) < 0.002:
                convergence = True

            # update the previous weights
            previous_w = self.w[:]

        return


    # make the prediction based on the model fitted before
    def predict(self, X):

        N = len(X)
        predictions = []

        # make predictions for xt, where t = 1, ..., n
        for t in range(N):

            # obtain the probability P(1 | xt)
            o = self.w[0] * 1

            for j in range(self.d):
                o += self.w[j+1] * X[t][j]

            prob_1 = np.exp(o) / (1 + np.exp(o))

            # make the prediction based on P(1 | xt)
            if prob_1 > 0.5:
                predictions.append(1)
            else:
                predictions.append(0)

        return predictions


    # The score function will return the accuracy of the model fitted before
    # using the test data X and y.

    def score(self, X, y):
        n = len(y)
        success = 0
        predict_y = self.predict(X)

        for i in range(n):
            if predict_y[i] == y[i]:
                success = success + 1

        return success / n



# 'my_cross_val()' function from the previous homework
# The main file
def my_cross_val(method, X, y, k):

    # Shuffle X and y together and then split them into k-groups
    split_result = shuffle_then_split(X, y, k)

    X_new = split_result[0]
    y_new = split_result[1]

    errors = []

    for i in range(k):
        X_train = make_list_except(X_new, i)
        y_train = make_list_except(y_new, i)
        X_test = X_new[i]
        y_test = y_new[i]

        method.fit(X_train, y_train)
        errors.append(1 - method.score(X_test, y_test))
        print("Fold " + str(i + 1) + ": " + str(errors[i]))

    print("Mean: " + str(np.mean(errors)))
    print("Standard Deviation: " + str(np.std(errors)) + "\n")

    return method


# Helper functions of my_cross_val()
def shuffle_then_split(X, y, k):

    # Shuffle the lists first to make the random split
    # Make the copies of X and y
    a = list(X)
    b = list(y)

    # Combine a and b into a list of tuples then shuffle
    combined = list(zip(a, b))
    np.random.shuffle(combined)

    # Get a and b back
    a[:], b[:] = zip(*combined)

    # Split the lists after shuffling into k-groups
    cutoff = len(a) / float(k)
    out1, out2 = [], []
    last = 0.0

    while last < len(a):
        out1.append(a[int(last):int(last + cutoff)])
        out2.append(b[int(last):int(last + cutoff)])
        last += cutoff

    return [out1, out2]


# This function is used to remove the test data to make the train set
def make_list_except(lst, i):
    result = []

    for j in range(len(lst)):
        if j != i:
            result.extend(lst[j])

    return result



def test_main():
    # digits = load_digits()
    # x = digits.data
    # y = digits.target
    
    # (1797, 64)
    # (1797,)
    # print(x.shape)
    # print(y.shape)
    
    # print()
    
    # <class 'numpy.ndarray'>
    # print(type(x))
    # print(type(y))

    with open('lttorb.txt') as f:
        check_lines = f.readlines()
    check_shapes = []
    for i in range(len(check_lines)):
        check_shapes.append(process_point(check_lines[i]))

    plt.imshow(torch.Tensor(make_plot(check_shapes[999])).numpy().squeeze(), cmap='gray_r')
    plt.show()
    print(len(check_shapes))

test_main()