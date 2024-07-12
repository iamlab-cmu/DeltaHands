import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from Kinematics_Delta import PDelta

def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in degrees.
    """
    ox, oy = origin
    px, py = point
    angle = np.radians(angle)

    qx = ox + np.cos(angle) * (px - ox) - np.sin(angle) * (py - oy)
    qy = oy + np.sin(angle) * (px - ox) + np.cos(angle) * (py - oy)
    return [qx, qy]


class DeltaHand():
    def __init__(self, num_finger=4, center_len=15, ee_len=6, base_len=15, leg_len=40):
        self.delta_centers = []
        self.delta_fingers = []

        self.num_finger = num_finger
        self.center_len = center_len
        self.ee_len = ee_len
        self.base_len = base_len
        self.leg_len = leg_len

        # starting from top -> left -> bottom -> right
        # all deltas facing downwards
        # delta in order bottom -> top right -> top left

        self.init_center = [0, 1*(self.center_len + self.base_len)]
        self.angles = np.linspace(0, -360, self.num_finger, endpoint=False) + 135
        self.init_centers = []
        for i in range(self.num_finger):
            finger = PDelta(ee_len=self.ee_len, base_len=self.base_len, leg_len=self.leg_len)
            center = rotate([0,0], self.init_center, self.angles[i])
            self.delta_centers.append(center)
            self.delta_fingers.append(finger)
            self.init_centers.append([finger.ix, finger.iy, finger.iz])

        # make it positive!!
        self.hand_init_center = np.mean(np.array(self.init_centers),axis=0)
        # self.hand_init_center[2] *= -1.0

    def go_to_FK(self, distances):
        # distances are actuation distances, and it has to be in shape (num_finger x 3)
        ee_positions = []
        for i in range(self.num_finger):
            x, y, z = self.delta_fingers[i].go_to_distance(distances[i][0],distances[i][1], distances[i][2])
            # rotate to the world coordinate
            x_t = x + self.init_center[0]
            y_t = y + self.init_center[1]
            x_r, y_r = rotate([0,0], [x_t, y_t], self.angles[i])
            ee_positions.append([x_r,y_r,z])
        return ee_positions

    def go_to_IK(self, positions):
        # positions are ee's positions, and it has to be in the shape (num_finger x 3 [x y z])
        actuations = []
        err = False
        for i in range(self.num_finger):
            pos = positions[i]
            # rotate to delta's coordinate
            pos_d = rotate([0,0], [pos[0], pos[1]], -1*self.angles[i])
            pos_x = pos_d[0] - self.init_center[0]
            pos_y = pos_d[1] - self.init_center[1]
            a1, a2, a3 = self.delta_fingers[i].go_to(pos_x, pos_y, pos[2])
            if a1 <= 0 or a2 <= 0 or a3 <= 0:
                err = True
            if a1 >= 20:
                a1 = 20.0
            if a2 >= 20:
                a2 = 20.0
            if a3 >= 20:
                a3 = 20.0
            actuations.append([a1, a2, a3])
        return actuations, err
