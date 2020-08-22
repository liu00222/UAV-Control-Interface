import numpy as np


class Triangle2D:
    def __init__(self, a, b, c):
        self.a, self.b, self.c = [a[0], a[1]], [b[0], b[1]], [c[0], c[1]]
        self.ab = np.array([b[0] - a[0], b[1] - a[1]])
        self.bc = np.array([c[0] - b[0], c[1] - b[1]])
        self.ca = np.array([a[0] - c[0], a[1] - c[1]])

    def contains(self, p):
        """
        Determine if the point p is inside the triangle

        :param p: <list> the point to be checked
        :return: <boolean>
        """
        inside = False
        pa = np.array([self.a[0] - p[0], self.a[1] - p[1]])
        pb = np.array([self.b[0] - p[0], self.b[1] - p[1]])
        pc = np.array([self.c[0] - p[0], self.c[1] - p[1]])

        # if the point is on the line ab or ca
        if np.cross(pa, self.ab) == 0 or np.cross(pa, self.ca) == 0:
            return True
        # if the point is on the line ab or bc
        if np.cross(pb, self.bc) == 0 or np.cross(pb, self.ab) == 0:
            return True
        # if the point is on the line bc or ca
        if np.cross(pc, self.ca) == 0 or np.cross(pc, self.bc) == 0:
            return True

        return (self.same_side(self.a, self.b, self.c, p) and
                self.same_side(self.b, self.c, self.a, p) and
                self.same_side(self.c, self.a, self.b, p))

    def same_side(self, a, b, c, p):
        """
        Helper function in deciding if p is inside of the triangle abc

        :param a: <list>
        :param b: <list>
        :param c: <list>
        :param p: <list>
        :return: <boolean>
        """
        ab = np.array([b[0] - a[0], b[1] - a[1]])
        ac = np.array([c[0] - a[0], c[1] - a[1]])
        ap = np.array([p[0] - a[0], p[1] - a[1]])

        v1 = np.cross(ab, ac)
        v2 = np.cross(ab, ap)

        return v1 * v2 >= 0


def my_rounding(num):
    """
    If the num's decimal part is >= 0.5, round it up; otherwise round it down

    :param num: <float> the number to be checked
    :return: <int> the rounded number
    """
    if num - int(num) >= 0.5:
        return int(num + 1)
    return int(num)
