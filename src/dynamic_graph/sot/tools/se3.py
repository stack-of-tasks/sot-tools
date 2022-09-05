# -*- coding: utf-8 -*-

# Copyright 2010 CNRS
# Author: Florent Lamiraux
#
# Release under LGPL license: see COPYING.LESSER at root of the project.
#

from functools import reduce
from math import cos, pi, sin, sqrt

import numpy


class R(object):
    def __init__(self, value):
        self.value = value

    def __mul__(self, other):
        if isinstance(other, R3):
            return other.multiplyByConstant(self.value)
        if isinstance(other, float):
            return R(self.value * other)
        if isinstance(other, R):
            return R(self.value * other.value)
        raise TypeError(
            "cannot multiply instances of type R and type " + str(type(other))
        )

    def __str__(self):
        return str(self.value)


class R3(object):
    """
    Vectors of real number of dimension 3.
    """

    def __init__(self, *args):
        if len(args) == 1:
            if isinstance(args[0], R3):
                # copy constructor
                self.value = args[0].value
            # constructor by tuple
            else:
                self.value = numpy.array(map(float, args[0]))
        elif len(args) == 3:
            self.value = numpy.array(map(float, args))
        else:
            raise TypeError(
                "constructor of R3 takes either three float,"
                + ", an instance of R or a tuple of three float."
            )
        self.check()

    def check(self):
        """
        Check that instance is well defined.
        """
        if len(self.value) != 3:
            raise TypeError("R3 object should be of length 3")

    def checkOther(self, other, operator):
        """
        Check that argument is of type R3
        """
        if not isinstance(other, R3):
            raise TypeError(
                "second argument of operator "
                + operator
                + " should be an instance of R3."
            )

    def crossprod(self, other):
        """
        Cross product
        """
        self.checkOther(other, "crossprod")
        cp = []
        cp.append(self.value[1] * other.value[2] - self.value[2] * other.value[1])
        cp.append(self.value[2] * other.value[0] - self.value[0] * other.value[2])
        cp.append(self.value[0] * other.value[1] - self.value[1] * other.value[0])
        return R3(tuple(cp))

    def __add__(self, other):
        """
        Operator +
        """
        self.checkOther(other, "+")
        return R3(tuple(map(lambda x: x[0] + x[1], zip(self, other))))

    def __sub__(self, other):
        """
        Operator -
        """
        self.checkOther(other, "-")
        return R3(tuple(map(lambda x: x[0] - x[1], zip(self, other))))

    def __mul__(self, other):
        """
        Operator *: inner product
        """
        self.checkOther(other, "*")
        return reduce(lambda x, y: x + y[0] * y[1], zip(self, other), 0.0)

    def __rmul__(self, number):
        """
        Operator * by a float
        """
        return R3(tuple(map(lambda x: number * x, self)))

    def __str__(self):
        """
        Output as a string
        """
        return "({0},{1},{2})".format(*self.value)

    def __getitem__(self, index):
        """
        Access by []
        """
        return self.value[index]

    def __setitem__(self, index, value):
        """
        Access by []
        """
        self.value[index] = value

    def multiplyByConstant(self, constant):
        """
        Multiply vector by a constant value
        """
        return R3(tuple(map(lambda x: constant * x, self)))

    def toTuple(self):
        return self.value

    def __array__(self):
        return numpy.array(self.value)


class SO3(object):
    """
    Rotation matrix
    """

    def __init__(self, matrix):
        if isinstance(matrix, SO3):
            self.value = matrix.value
        else:
            self.value = self.fromTuple(matrix)

    def fromTuple(self, matrix):
        if len(matrix) != 3:
            raise TypeError(
                "expecting a tuple of "
                + "3 tuples of 3 float, got a tuple of length %i." % len(matrix)
            )
        m = []
        for r in matrix:
            if len(r) != 3:
                raise TypeError("expecting tuple of 3 float or instance of R3.")
            m.append(R3(r[0], r[1], r[2]))
        return tuple(m)

    def inverse(self):
        """
        Return the inverse of the matrix
        """
        return self.transpose()

    def transpose(self):
        """
        Return the transpose of the matrix
        """
        return SO3(tuple(zip(*self)))

    def __str__(self):
        """
        Output as a string
        """
        return "(%s,%s,%s)" % self.value

    def __getitem__(self, index):
        """
        Access by []
        """
        return self.value[index]

    def __mul__(self, other):
        """
        Operator *

          second argument is either
            - an instance of SO3 or
            - an instance of R3.
        """
        if isinstance(other, SO3):
            return self.multiplyBySO3(other)
        if isinstance(other, R3):
            return self.multiplyByR3(other)
        raise TypeError(
            "Cannot multiply instance of SO3 by instance of " + str(type(other)) + "."
        )

    def multiplyByR3(self, other):
        return R3(self[0] * other, self[1] * other, self[2] * other)

    def multiplyBySO3(self, other):
        return SO3(
            reduce(lambda m, col: m + (tuple(self * col),), other.transpose(), ())
        ).transpose()


class SE3(object):
    def __init__(self, *args):
        """
        Constructor
          - by rotation matrix and translation vector,
          - by homogeneous matrix
          - without argument: return identity matrix
        """
        if len(args) == 2:
            # expecting translation and rotation
            self.rotation = SO3(args[0])
            self.translation = R3(args[1])
        elif len(args) == 1:
            # expecting homogeneous matrix
            matrix = args[0]
            self.rotation = SO3(tuple(map(lambda row: row[:3], matrix[:3])))
            self.translation = R3(tuple(map(lambda row: row[3], matrix[:3])))
        elif len(args) == 0:
            self.rotation = SO3(
                ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            )
            self.translation = R3(0.0, 0.0, 0.0)
        else:
            raise ValueError("two many arguments.")

    def __str__(self):
        """
        Output as a string
        """
        return "((%f,%f,%f,%f),(%f,%f,%f,%f),(%f,%f,%f,%f),(0.,0.,0.,1.))" % (
            self.rotation[0][0],
            self.rotation[0][1],
            self.rotation[0][2],
            self.translation[0],
            self.rotation[1][0],
            self.rotation[1][1],
            self.rotation[1][2],
            self.translation[1],
            self.rotation[2][0],
            self.rotation[2][1],
            self.rotation[2][2],
            self.translation[2],
        )

    def inverse(self):
        trRot = self.rotation.transpose()
        return SE3(trRot, R(-1.0) * (trRot * self.translation))

    def __mul__(self, other):
        """
        Operator *

          other argument is either
            - an instance of SE3 or,
            - an instance of R3.
        """
        if isinstance(other, SE3):
            return self.multiplyBySE3(other)
        if isinstance(other, R3):
            return self.multiplyByR3(other)
        raise TypeError(
            "cannot multiply instance of SE3 by instance of " + str(type(other))
        )

    def multiplyBySE3(self, other):
        return SE3(
            self.rotation * other.rotation,
            self.rotation * other.translation + self.translation,
        )

    def multiplyByR3(self, other):
        return R3(self.rotation * other + self.translation)

    def __getitem__(self, index):
        if index == 3:
            return (0.0, 0.0, 0.0, 1.0)
        else:
            return tuple(self.rotation[index]) + (self.translation[index],)

    def toMatrix(self):
        r = self.rotation
        t = self.translation
        return (
            (r[0][0], r[0][1], r[0][2], t[0]),
            (r[1][0], r[1][1], r[1][2], t[1]),
            (r[2][0], r[2][1], r[2][2], t[2]),
            (0.0, 0.0, 0.0, 1.0),
        )


if __name__ == "__main__":
    a = R(2.0)
    u = R3(1.0, 2.0, 3.0)
    v = R3(2.0, 3.0, 4.0)
    print("a = " + str(a))
    print("u = " + str(u))
    print("v = " + str(v))
    print("a*a = " + str(a * a))
    print("a*u = " + str(a * u))
    print("u*u = " + str(u * u))
    print("u-v = " + str(u - v))
    print("u*v = " + str(u * v))

    for i, x in zip(range(3), u):
        print("u[%i] = %f" % (i, x))

    rot = (
        (cos(pi / 6), -sin(pi / 6), 0.0),
        (sin(pi / 6), cos(pi / 6), 0.0),
        (0.0, 0.0, 1.0),
    )
    m1 = SO3(rot)
    print("m1 = " + str(m1))
    print("")
    rot = (
        (cos(pi / 6), 0.0, -sin(pi / 6)),
        (0.0, 1.0, 0.0),
        (sin(pi / 6), 0.0, cos(pi / 6)),
    )

    m2 = SO3(rot)
    print("m2 = " + str(m2))
    print("")

    print("m1*m2 = " + str(m1 * m2))
    print(
        "should be ((%f,%f,%f),(%f,%f,%f),(%f,%f,%f))"
        % (
            0.75,
            -0.5,
            -sqrt(3) / 4,
            sqrt(3) / 4,
            sqrt(3) / 2,
            -0.25,
            0.5,
            0.0,
            sqrt(3) / 2,
        )
    )
    print("")
    m3 = SE3(m1 * m2, u)
    print("m3 = " + str(m3))
    print("")

    for i in range(4):
        for j in range(4):
            print("m3[%i][%i] = %f" % (i, j, m3[i][j]))
    print("")

    m3inv = m3.inverse()
    print("m3.inverse() = " + str(m3inv))
    print("")

    print("m3inv * m3 = " + str(m3inv * m3))
    print("")
    print("m3 * m3inv = " + str(m3 * m3inv))
