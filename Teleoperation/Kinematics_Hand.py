import numpy as np
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
            #print("Go to: ", pos_x, pos_y, pos[2])
            a1, a2, a3 = self.delta_fingers[i].go_to(pos_x, pos_y, pos[2])
            #print("Actuation to: ", [a1, a2, a3])
            # if a1 >= 20.0 or a2 >= 20.0 or a3 >= 20.0 or a1 <= 0 or a2 <= 0 or a3 <= 0:
            if a1 <= 0 or a2 <= 0 or a3 <= 0:
                # print("Outrange in IK!")
                # pause = input("pause for err")
                err = True
            if a1 >= 20:
                a1 = 20.0
            if a2 >= 20:
                a2 = 20.0
            if a3 >= 20:
                a3 = 20.0
            actuations.append([a1, a2, a3])
        return actuations, err

if __name__ == "__main__":
    hand = DeltaHand()
    print(hand.delta_centers)
    # actuations = [[0,0,0], [20,20,20], [5,5,5], [10,10,10]]
    # ee_pos = hand.go_to_FK(actuations)
    # print(ee_pos)
    steps = 10
    middle_steps = 4
    middle_range = np.linspace(0, 20, middle_steps, endpoint=True)
    for motor1_act in middle_range:
        print(motor1_act)
        motor1_range = [motor1_act]
        motor2_range = np.linspace(0, 20, steps, endpoint=True)
        motor3_range = np.linspace(0, 20, steps, endpoint=True)
        x_solutions = [[],[],[],[]]
        y_solutions = [[],[],[],[]]
        z_solutions = [[],[],[],[]]
        errs = []

        for i, dst1 in enumerate(motor1_range):
            for j, dst2 in enumerate(motor2_range):
                for k, dst3 in enumerate(motor3_range):
                    # print("# Distance: ", dst1, dst2, dst3)
                    actuations = [[dst1, dst2, dst3], [dst1, dst2, dst3], [dst1, dst2, dst3], [dst1, dst2, dst3]]
                    ee_pos = hand.go_to_FK(actuations)
                    # print(ee_pos)
                    for e in range(hand.num_finger):
                        x_solutions[e].append(ee_pos[e][0])
                        y_solutions[e].append(ee_pos[e][1])
                        z_solutions[e].append(ee_pos[e][2])

        x_solutions = np.array(x_solutions) #* 0.001
        y_solutions = np.array(y_solutions) #* 0.001
        z_solutions = np.array(z_solutions) * -1 #* 0.001 * -1
        z_solutions -= np.min(z_solutions)

        print("Min/Max on x axis: ", np.min(x_solutions), np.max(x_solutions))
        print("Min/Max on y axis: ", np.min(y_solutions), np.max(y_solutions))
        print("Min/Max on z axis: ", np.min(z_solutions), np.max(z_solutions))
