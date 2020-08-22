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



NUM_PIXEL = 27


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
    
    with open('data/s.txt') as f:
        s_lines = f.readlines()
    s_shapes = []
    for i in range(len(s_lines)):
        s_shapes.append(process_point(s_lines[i]))

    with open('data/check.txt') as f:
        check_lines = f.readlines()
    check_shapes = []
    for i in range(len(check_lines)):
        check_shapes.append(process_point(check_lines[i]))

    with open('data/check_reverse.txt') as f:
        check_reverse_lines = f.readlines()
    check_reverse_shapes = []
    for i in range(len(check_reverse_lines)):
        check_reverse_shapes.append(process_point(check_reverse_lines[i]))
    
    with open('data/horizontal.txt') as f:
        horizontal_lines = f.readlines()
    horizontal_shapes = []
    for i in range(len(horizontal_lines)):
        horizontal_shapes.append(process_point(horizontal_lines[i]))
    
    with open('data/vertical.txt') as f:
        vertical_lines = f.readlines()
    vertical_shapes = []
    for i in range(len(vertical_lines)):
        vertical_shapes.append(process_point(vertical_lines[i]))
    
    with open('data/lttorb.txt') as f:
        lttorb_lines = f.readlines()
    lttorb_shapes = []
    for i in range(len(lttorb_lines)):
        lttorb_shapes.append(process_point(lttorb_lines[i]))
    
    with open('data/lbtort.txt') as f:
        lbtort_lines = f.readlines()
    lbtort_shapes = []
    for i in range(len(lbtort_lines)):
        lbtort_shapes.append(process_point(lbtort_lines[i]))

    # plt.imshow(torch.Tensor(make_plot(check_shapes[1])).numpy().squeeze(), cmap='gray_r')
    # plt.show()
    # print(len(lines))

    raw_train_x = []
    raw_train_y = []
    raw_val_x = []
    raw_val_y = []

    # 0 : s
    # 1 : check
    # 2 : check_reverse
    for i in range(len(s_lines)):
        if 0 <= i <= len(s_lines) - 101:
            raw_train_x.append(make_plot(s_shapes[i]))
            raw_train_y.append(0)
        else:
            raw_val_x.append(make_plot(s_shapes[i]))
            raw_val_y.append(0)

    for j in range(len(check_lines)):
        if 0 <= j <= len(check_lines) - 101:
            raw_train_x.append(make_plot(check_shapes[j]))
            raw_train_y.append(1)
        else:
            raw_val_x.append(make_plot(check_shapes[j]))
            raw_val_y.append(1)

    for k in range(len(check_reverse_lines)):
        if 0 <= k <= len(check_reverse_lines) - 101:
            raw_train_x.append(make_plot(check_reverse_shapes[k]))
            raw_train_y.append(2)
        else:
            raw_val_x.append(make_plot(check_reverse_shapes[k]))
            raw_val_y.append(2)
            
    for k in range(len(horizontal_lines)):
        if 0 <= k <= len(horizontal_lines) - 101:
            raw_train_x.append(make_plot(horizontal_shapes[k]))
            raw_train_y.append(3)
        else:
            raw_val_x.append(make_plot(horizontal_shapes[k]))
            raw_val_y.append(3)
        
    for k in range(len(vertical_lines)):
        if 0 <= k <= len(vertical_lines) - 101:
            raw_train_x.append(make_plot(vertical_shapes[k]))
            raw_train_y.append(4)
        else:
            raw_val_x.append(make_plot(vertical_shapes[k]))
            raw_val_y.append(4)
    
    for k in range(len(lbtort_lines)):
        if 0 <= k <= len(lbtort_lines) - 101:
            raw_train_x.append(make_plot(lbtort_shapes[k]))
            raw_train_y.append(5)
        else:
            raw_val_x.append(make_plot(lbtort_shapes[k]))
            raw_val_y.append(5)
        
    for k in range(len(lttorb_lines)):
        if 0 <= k <= len(lttorb_lines) - 101:
            raw_train_x.append(make_plot(lttorb_shapes[k]))
            raw_train_y.append(6)
        else:
            raw_val_x.append(make_plot(lttorb_shapes[k]))
            raw_val_y.append(6)
    

    x_train = (np.array(raw_train_x)).reshape(6300, 729)
    y_train = np.array(raw_train_y)
    
    x_test = ((np.array(raw_val_x)).reshape(700, 729)).tolist()
    y_test = np.array(raw_val_y)
    
    
    model = RandomForestClassifier()
    model.fit(x_train, y_train)

    # convert to pure python estimator
    clf_pure_predict = convert_estimator(model)
    with open("model.pkl", "wb") as f:
        pickle.dump(clf_pure_predict, f)
    
    
    print(model.score(x_test, y_test))
    
    
    # load pickled model
    with open("model.pkl", "rb") as f:
        clf = pickle.load(f)

    # make prediction with pure-predict object
    y_pred = clf.predict([x_test[0]])
    print(y_pred)
    print(y_test[0])

test_main()