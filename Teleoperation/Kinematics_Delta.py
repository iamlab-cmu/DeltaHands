#!/usr/bin/env python

import time
import math
import numpy as np

PI = math.pi

class PDelta():
    def __init__(self, ee_len=6, base_len=15, leg_len=40):
        # all the units are in milimeter, or degrees
        self.e = ee_len * 2 * np.sqrt(3) #25 # end effector's side length
        self.f = base_len * 2 * np.sqrt(3) #50 # base's side length
        self.re = leg_len #60 # leg's length (the one connected to EE)
        self.t1, self.t2, self.t3 = 0, 0, 0
        self.x0, self.x1, self.x2 = 0, 0, 0

        # constants
        self.sin120 = np.sqrt(3)/2.0
        self.cos120 = -0.5
        self.tan60 = np.sqrt(3)
        self.sin30 = 0.5
        self.cos30 = np.sqrt(3) / 2
        self.tan30 = 1 / np.sqrt(3)
        self.ix, self.iy, self.iz = self.go_home()

    def report_distance(self):
        print('AT DISTANCE {:0.2f}, {:0.2f}, {:0.2f}'.format(self.t1, self.t2, self.t3))

    def report_position(self):
        print('AT POS {:0.2f}, {:0.2f}, {:0.2f}'.format(self.x0, self.y0, self.z0))

    def go_home(self):
        # TODO: define z_safe properly
        ix, iy, iz = self.go_to_distance(0, 0, 0)
        # self.report_position()
        return ix, iy, iz

    def go_to(self, x, y, z):
        # give a position, use IK to calculate angles of Motors and actuate them
        self.x0, self.y0, self.z0 = x, y, z
        state = self.delta_calc_inverse()
        if state == 0:
            # self.report_distance()
            pass
        else:
            print("Err in IK!")
            # pause = input("pause for err")
            # self.t1, self.t2, self.t3 = 0, 0, 0
        return self.t1, self.t2, self.t3

    def go_to_distance(self, dst_1, dst_2, dst_3):
        # give distance of motors, use FK to calculate the EE's position
        x_old, y_old, z_old = self.x0, self.x1, self.x2
        # Set x,y,z from new distance
        stat = self.delta_calc_forward(dst_1, dst_2, dst_3)
        if stat == 0:
            self.t1, self.t2, self.t3 = dst_1, dst_2, dst_3
            # self.report_position()
        else:
            print("Err in FK!")
            pause = input("pause for err")
        return self.x0, self.y0, self.z0

    def delta_calc_inverse(self):
        # find corners of ee platform
        t = self.e * self.tan30 / 2
        position = np.array([self.x0, self.y0, self.z0])
        c1 = position + np.array([0, -1*t, 0])
        c2 = position + np.array([t * self.cos30, t * self.sin30, 0])
        c3 = position + np.array([-1 * t * self.cos30, t * self.sin30, 0])

        # find the corners of base
        f = self.f * self.tan30 / 2
        base1 = np.array([0, -1*f, 0])
        base2 = np.array([f * self.cos30, f * self.sin30, 0])
        base3 = np.array([-1 * f * self.cos30, f * self.sin30, 0])

        # squared dist to prismatic actuator axis
        d1 = np.sum(np.power(c1[0:2] - base1[0:2], 2))
        d2 = np.sum(np.power(c2[0:2] - base2[0:2], 2))
        d3 = np.sum(np.power(c3[0:2] - base3[0:2], 2))

        l = np.power(self.re, 2)
        if d1 > l or d2 > l or d3 > l:
            return -1

        self.t1 = -1*(position[2] + np.sqrt(l - d1))
        self.t2 = -1*(position[2] + np.sqrt(l - d2))
        self.t3 = -1*(position[2] + np.sqrt(l - d3))
        return 0

    def delta_calc_forward(self, dst_1, dst_2, dst_3):
        # Calculate forward positions given the distance.
        t = (self.f - self.e) * self.tan30 / 2

        y1 = -1 * t
        z1 = -1 * dst_1

        y2 = t * self.sin30
        x2 = t * self.cos30
        z2 = -1 * dst_2

        y3 = t * self.sin30
        x3 = -t * self.cos30
        z3 = -1 * dst_3

        dnm = (y2 - y1) * x3 - (y3 - y1) * x2

        w1 = y1 * y1 + z1 * z1
        w2 = x2 * x2 + y2 * y2 + z2 * z2
        w3 = x3 * x3 + y3 * y3 + z3 * z3

        # x = (a1*z + b1)/dnm
        a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
        b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0

        # y = (a2*z + b2)/dnm
        a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
        b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

        # a*z^2 + b*z + c = 0
        a = a1 * a1 + a2 * a2 + dnm * dnm
        b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm)
        c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - self.re * self.re)

        # discriminant
        d = b * b - (4.0 * a * c)
        if (d < 0):
            return -1  # non-existing point

        self.z0 = -(0.5 * (b + np.sqrt(d)) / a)
        self.x0 = (a1 * self.z0 + b1) / dnm
        self.y0 = (a2 * self.z0 + b2) / dnm

        return 0
