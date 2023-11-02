import os

import taichi as ti
from os import path as osp
from datetime import datetime
import time
import csv
import cv2
import numpy as np
import matplotlib.pyplot as plt
from Kinematics_Hand import DeltaHand
from DeltaHand import DeltaHandEnv

import rospy
import datetime
import signal
from leap_motion_msgs.msg import *
import argparse


class LeapMotionLogger:
    def __init__(self, record_data, data_save_dir):
        self.leap_sub = rospy.Subscriber("/leap_motion", LeapData, self.callback)
        print("leap motion sensor subscribed")
        self.sliding_window_size = 20
        self.data_array = np.zeros((self.sliding_window_size,18))
        self.data_record = record_data
        self.data_save_dir = data_save_dir
        self.current_data_idx = 0

        if record_data:
            self.csv_file = open(self.data_save_dir, 'w')
            self.csv_writer = csv.writer(self.csv_file)
            # h1 = left hand, h2 = right hand, f1 = thumb, f2 = index finger
            header = ['timestamp','h1x', 'h1y', 'h1z', 'h1f1x', 'h1f1y', 'h1f1z', 'h1f2x', 'h1f2y', 'h1f2z','h2x', 'h2y', 'h2z', 'h2f1x', 'h2f1y', 'h2f1z', 'h2f2x', 'h2f2y', 'h2f2z']
            self.csv_writer.writerow(header)

        self.init_hand = []
        self.init_cnt = 0
        self.init_num = 50

        time.sleep(1)

        while(self.init_cnt < self.init_num):
            avg_array = np.mean(self.data_array,axis=0)
            self.init_hand.append(avg_array)
            self.init_cnt += 1

        self.init_hand = np.array(self.init_hand)

        # left thumb
        init_finger_1 = np.zeros((self.init_num,3))
        init_finger_1[:,0] = self.init_hand[:,0+3]
        init_finger_1[:,1] = -self.init_hand[:,2+3]
        init_finger_1[:,2] = 0

        # left index
        init_finger_2 = np.zeros((self.init_num,3))
        init_finger_2[:,0] = self.init_hand[:,0+6]
        init_finger_2[:,1] = -self.init_hand[:,2+6]
        init_finger_2[:,2] = 0

        # right thumb
        init_finger_4 = np.zeros((self.init_num,3))
        init_finger_4[:,0] = self.init_hand[:,0+12]
        init_finger_4[:,1] = -self.init_hand[:,2+12]
        init_finger_4[:,2] = 0

        # right index
        init_finger_3 = np.zeros((self.init_num,3))
        init_finger_3[:,0] = self.init_hand[:,0+15]
        init_finger_3[:,1] = -self.init_hand[:,2+15]
        init_finger_3[:,2] = 0

        self.center_of_fingers = 0.25 *np.mean(init_finger_1[:100,:] + init_finger_2[:100,:] + init_finger_3[:100,:] + init_finger_4[:100,:], axis=0)
        print(self.center_of_fingers)
        self.init_cnt = 0
        self.init_hand = []

    def callback(self, data):
        current_data_array = np.zeros((18))

        for hand in data.hands:
            if hand.is_left:
                for i in range(3):
                    current_data_array[i] = hand.hand_position[i]

                for finger in hand.fingers:
                    if finger.finger_type == "Thumb":
                        for i in range(3):
                            current_data_array[i+3] = finger.bones[3].end_position[i]
                    elif finger.finger_type == "Index":
                        for i in range(3):
                            current_data_array[i+6] = finger.bones[3].end_position[i]
            else:
                for i in range(3):
                    current_data_array[i+9] = hand.hand_position[i]

                for finger in hand.fingers:
                    if finger.finger_type == "Thumb":
                        for i in range(3):
                            current_data_array[i+12] = finger.bones[3].end_position[i]
                    elif finger.finger_type == "Index":
                        for i in range(3):
                            current_data_array[i+15] = finger.bones[3].end_position[i]

        if np.count_nonzero(current_data_array) == 18:
            self.data_array[self.current_data_idx,:] = current_data_array
            self.current_data_idx += 1
            self.current_data_idx = self.current_data_idx % self.sliding_window_size

            if self.data_record:
                timestamp = time.time()
                data = [timestamp] + current_data_array.tolist()
                self.csv_writer.writerow(data)

    def startRecord(self):
        self.data_record = True

    def stopRecord(self):
        self.data_record = False

    def terminateRecord(self):
        self.csv_file.close()

    def get_curr_reading(self):
        return self.data_array

    # 0 y 1 z 2 x

    def process_data(self, offset, height):
        avg_array = np.mean(self.data_array,axis=0)
        # left thumb
        finger_1 = np.zeros((3))
        finger_1[0] = avg_array[0+3]
        finger_1[1] = -avg_array[2+3]
        finger_1[2] = (avg_array[1+3] - avg_array[1+0] + 3) * 0.15 + height

        # left index
        finger_2 = np.zeros((3))
        finger_2[0] = avg_array[0+6]
        finger_2[1] = -avg_array[2+6]
        finger_2[2] = (avg_array[1+6] - avg_array[1+0] + 3) * 0.15 + height

        # right thumb
        finger_4 = np.zeros((3))
        finger_4[0] = avg_array[0+12]
        finger_4[1] = -avg_array[2+12]
        finger_4[2] = (avg_array[1+12] - avg_array[1+9] + 3) * 0.15 + height

        # right index
        finger_3 = np.zeros((3))
        finger_3[0] = avg_array[0+15]
        finger_3[1] = -avg_array[2+15]
        finger_3[2] = (avg_array[1+15] - avg_array[1+9] + 3) * 0.15 + height
        print("Height offset: ", (avg_array[1+6] - avg_array[1+0] + 3) * 0.15 , (avg_array[1+15] - avg_array[1+9] + 3) * 0.15)

        if(self.init_cnt < self.init_num):
            self.init_hand.append(avg_array)
            self.init_cnt += 1

        if(self.init_cnt == self.init_num):
            self.init_hand = np.array(self.init_hand)

            # left thumb
            init_finger_1 = np.zeros((self.init_num,3))
            init_finger_1[:,0] = self.init_hand[:,0+3]
            init_finger_1[:,1] = -self.init_hand[:,2+3]
            init_finger_1[:,2] = 0

            # left index
            init_finger_2 = np.zeros((self.init_num,3))
            init_finger_2[:,0] = self.init_hand[:,0+6]
            init_finger_2[:,1] = -self.init_hand[:,2+6]
            init_finger_2[:,2] = 0

            # right thumb
            init_finger_4 = np.zeros((self.init_num,3))
            init_finger_4[:,0] = self.init_hand[:,0+12]
            init_finger_4[:,1] = -self.init_hand[:,2+12]
            init_finger_4[:,2] = 0

            # right index
            init_finger_3 = np.zeros((self.init_num,3))
            init_finger_3[:,0] = self.init_hand[:,0+15]
            init_finger_3[:,1] = -self.init_hand[:,2+15]
            init_finger_3[:,2] = 0

            self.center_of_fingers = 0.25 *np.mean(init_finger_1[:100,:] + init_finger_2[:100,:] + init_finger_3[:100,:] + init_finger_4[:100,:], axis=0)

            self.init_cnt = 0
            self.init_hand = []

        finger_offset_angles = np.deg2rad([-135, 135, 45, -45])
        self.fingers = [finger_1, finger_2, finger_3, finger_4]

        for i in range(4):
            self.fingers[i] -= self.center_of_fingers
            self.fingers[i][0] -= offset*np.cos(finger_offset_angles[i])
            self.fingers[i][1] -= offset*np.sin(finger_offset_angles[i])
            #self.fingers[i][2] = height
        return self.fingers


class GracefulExiter():

    def __init__(self):
        self.state = False
        signal.signal(signal.SIGINT, self.change_state)

    def change_state(self, signum, frame):
        print("exit flag set to True (repeat to exit now)")
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        self.state = True

    def exit(self):
        return self.state

ti.init(arch=ti.gpu)

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--record', '-r', action='store_true')
    parser.add_argument('--save_dir', '-s', default='')
    args = parser.parse_args()

    print("sleeping for 3 seconds...")
    time.sleep(3.0)

    flag = GracefulExiter()

    hand = DeltaHand(num_finger=4, center_len=15, ee_len=6, base_len=15, leg_len=42)
    height = 1 *  hand.hand_init_center[2]# - 5.0
    offset = 15 # mm

    env = DeltaHandEnv('/dev/ttyACM0', 57600)

    rospy.init_node("leap_motion_logger")
    leap_motion_logger = LeapMotionLogger(args.record, args.save_dir)

    gui = ti.GUI("Finger pos visualizer", (1024, 1024))

    cnt = 0
    err_cnt = 0
    send_command = False

    while gui.running:
        for e in gui.get_events(gui.PRESS):
            if e.key == gui.ESCAPE:
                gui.running = False
            ## add motions to 4 finger in mm
            ## finger 1
            elif e.key == "t":
                send_command = True
            elif e.key == "s":
                send_command = False

        # print(leap_motion_logger.get_curr_reading())
        time.sleep(0.05)
        # cur_ee = leap_motion_logger.get_curr_reading()
        desired_pos = leap_motion_logger.process_data(offset, height)

        act, err = hand.go_to_IK(desired_pos)
        if err:
            err_cnt += 1
        middle_act = 0.25*(act[0][0] + act[1][0] + act[2][0] + act[3][0])

        act[0][0] = middle_act
        act[1][0] = middle_act
        act[2][0] = middle_act
        act[3][0] = middle_act

        fk_pos = hand.go_to_FK(act) # 4 x 3
        # * 0.001
        traj_mm = np.array([middle_act, act[0][2], act[0][1], act[1][2], act[1][1], act[2][2], act[2][1], act[3][2], act[3][1]])
        traj = traj_mm * 0.001 # meter
        if send_command:
            env.move_joint_position([traj], [1.0])

        draw_fk_pos = np.array(fk_pos)[:,0:2]  * 0.01 + 0.5
        draw_desired_pos = np.array(desired_pos)[:,0:2]  * 0.01 + 0.5
        gui.circles(draw_fk_pos, radius=20, color=0x9403fc)
        gui.circles(draw_desired_pos, radius=20, color=0xFFAA33)
        gui.show()

        if flag.exit():
            break
