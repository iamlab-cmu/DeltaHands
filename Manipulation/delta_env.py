import os
from os import path as osp
from datetime import datetime
import time
import cv2
import numpy as np
from Kinematics_Hand import DeltaHand

import rospy
from std_msgs.msg import Float32MultiArray, Int16MultiArray
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage


class DeltaEnv:
    def __init__(self, enable_delta, enable_camera, reset_delta):

        self.enable_delta = enable_delta
        self.enable_camera = enable_camera
        self.reset_delta = reset_delta

        self.num_motors = 12
        self.min_pos = 0.001
        self.max_pos = 0.019
        self.joint_positions = [self.min_pos] * self.num_motors
        self.image = None

        self.hand_kinematic = DeltaHand(num_finger=4, center_len=15, ee_len=6, base_len=20, leg_len=42)

        if self.enable_delta:
            self.delta_pub = rospy.Publisher('/deltaControl', Float32MultiArray, queue_size=100)
            self.delta_sub = rospy.Subscriber('/deltaJointState', Float32MultiArray, self.jointCallback)
            print("delta hand joint state subscribed")
            time.sleep(1.0)

            if self.reset_delta:
                self.reset()
                time.sleep(1.0)

        if self.enable_camera:

            self.br = CvBridge()
            self.img_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.imgCallback)
            print("delta hand camera subscribed")

        time.sleep(1)
        print("Initialized Delta hand!")


    def jointCallback(self, joint_msg):
        for i in range(self.num_motors):
            self.joint_positions[i] = joint_msg.data[i]

    def imgCallback(self, img):
        # 480 x 640 x 3
        np_arr = np.frombuffer(img.data, np.uint8)
        self.image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def step(self, action):
        ## action in meter
        action = np.clip(action, self.min_pos, self.max_pos).tolist()
        control_msg = Float32MultiArray()
        # reset 0/1, stop 0/1 start, 0/1, position_control 0/1, target potisions # num_motors
        assert len(action) == self.num_motors
        control_flag = [0, 0, 0, 1]
        control_positions = action.copy()

        control_msg.data = control_flag + control_positions
        self.delta_pub.publish(control_msg)
        # return joint state and image after step as the new observation
        if self.enable_camera:
            img, oimg = self.get_image()
        else:
            img, oimg = None, None

        return {'image': img, 'agent_pos': action, 'oimage': oimg}


    def get_image(self):
        cur_image = self.image.copy()
        viz_image = self.image.copy()
        cur_image = cv2.rotate(cur_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cur_image = cv2.rotate(cur_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        res_img = cv2.resize(cur_image, dsize=(320, 240)) # (120, 160) (320, 240)
        return res_img, viz_image


    def reset(self):
        control_msg = Float32MultiArray()

        # reset 0/1, stop 0/1 start, 0/1, position_control 0/1, target potisions # num_motors
        control_flag = [0, 0, 0, 1]
        control_positions = [self.min_pos] * self.num_motors
        control_msg.data = control_flag + control_positions
        self.delta_pub.publish(control_msg)

        time.sleep(0.1)
