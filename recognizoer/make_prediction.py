import pickle
import numpy as np

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
    
    with open('s.txt') as f:
        s_lines = f.readlines()
    s_shapes = []
    for i in range(len(s_lines)):
        s_shapes.append(process_point(s_lines[i]))

    with open('check.txt') as f:
        check_lines = f.readlines()
    check_shapes = []
    for i in range(len(check_lines)):
        check_shapes.append(process_point(check_lines[i]))

    with open('check_reverse.txt') as f:
        check_reverse_lines = f.readlines()
    check_reverse_shapes = []
    for i in range(len(check_reverse_lines)):
        check_reverse_shapes.append(process_point(check_reverse_lines[i]))

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
    

    x_train = (np.array(raw_train_x)).reshape(2700, 729)
    y_train = np.array(raw_train_y)
    
    x_test = ((np.array(raw_val_x)).reshape(300, 729)).tolist()
    y_test = np.array(raw_val_y)
    
    
    # load pickled model
    with open("model.pkl", "rb") as f:
        clf = pickle.load(f)

    # make prediction with pure-predict object
    y_pred = clf.predict([x_test[0]])
    print(y_pred)
    print(y_test[0])

test_main()