import os
from os import path as osp
import time
import cv2
import rosbag
import numpy as np
import pickle as pkl
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int16MultiArray, Float32MultiArray
import argparse


class RosbagReader:
    def __init__(self, bag_path, file_name, save_path, file_time):
        self.bag_path = bag_path + file_name
        self.file_name = file_name[:-4]
        self.file_time = file_time
        self.save_path = save_path
        os.makedirs(self.save_path, exist_ok=True)

        self.bag = rosbag.Bag(self.bag_path)

        self.deltaControl_data = [] # all delta hand motors' joint positions
        self.deltaControl_time = []

        self.handControl_data = []
        self.handControl_time = []

        self.deltaJoint_data = []
        self.deltaJoint_time = []

        self.inhandImage_data = []
        self.inhandImage_time = []

        self.externalImage_data = []
        self.externalImage_time = []

        self.downsample_rate = 2 # downsaple data

    def read_deltaControl(self):
        cnt = 0
        for topic, msg, t in self.bag.read_messages(topics=['/deltaControl']):
            self.deltaControl_data.append(msg.data[4:])
            self.deltaControl_time.append(t)
            cnt += 1
        print("# delta control data: ", cnt)

    def read_handControl(self):

        cnt = 0
        for topic, msg, t in self.bag.read_messages(topics=['/slider']):
            self.handControl_data.append(msg.data)
            self.handControl_time.append(t)
            cnt += 1
        print("# slider data: ", cnt)

    def read_deltaJoints(self):

        cnt = 0
        for topic, msg, t in self.bag.read_messages(topics=['/deltaJointState']):
            ## downsample traj
            if cnt % self.downsample_rate == 0:
                self.deltaJoint_data.append(msg.data)
                self.deltaJoint_time.append(t)
            cnt += 1
        print("# delta joint data: ", cnt)

    def read_inhandImage(self):

        cnt = 0
        for topic, msg, t in self.bag.read_messages(topics=['/raspicam_node/image/compressed']):
            np_arr = np.frombuffer(msg.data, np.uint8)
            cur_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cur_image = cv2.rotate(cur_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            cur_image = cv2.rotate(cur_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            cur_image = cv2.resize(cur_image, dsize=(320, 240)) # dsize=(160, 120)
            self.inhandImage_data.append(cur_image)
            self.inhandImage_time.append(t)

            cnt += 1
        print("# in hand cam data: ", cnt)

    def read_externalImage(self):

        cnt = 0
        for topic, msg, t in self.bag.read_messages(topics=['/rgb/image_raw']):
            np_arr = np.frombuffer(msg.data, np.uint8).reshape(720, 1280, 3)

            cy = int(720/2)
            cx = int(1280/2 + 60)
            dy = int(240/2)
            dx = int(320/2)

            cropped_arr = np_arr[ cy-dy:cy+dy, cx-dx:cx+dx, :]
            self.externalImage_data.append(cur_image)
            self.externalImage_time.append(t)
            cnt += 1

        print("# ext cam data: ", cnt)

    def save_pkl(self):
        # save data
        self.data_filename = self.save_path + self.file_name + ".pkl"
        print("File name: ", self.file_name + ".pkl")

        self.pkl_data = {'deltaJointState': [],
                        'deltaControl': [],
                        'inHandImg': [],
                        'externalImg': []}

        ## sync data
        joint_time = np.array(self.deltaJoint_time)
        image_time = np.array(self.inhandImage_time)
        extimg_time = np.array(self.externalImage_time)
        control_time = np.array(self.deltaControl_time)


        cnt = 0
        ## sync the time
        synced_idx = []
        for i in range(len(joint_time)):
            while (cnt < len(image_time) and image_time[cnt]< joint_time[i]):
                cnt += 1
                continue
            synced_idx.append(cnt)


        cnt = 0
        ## sync the time
        synced_idx2 = []
        for i in range(len(joint_time)):
            while (cnt < len(extimg_time) and extimg_time[cnt]< joint_time[i]):
                cnt += 1
                continue
            synced_idx2.append(cnt)


        cnt = 0
        ## sync the time
        synced_idx3 = []
        for i in range(len(joint_time)):
            while (cnt < len(control_time) and control_time[cnt]< joint_time[i]):
                cnt += 1
                continue
            synced_idx3.append(cnt)


        print("# sync: ", len(joint_time), len(synced_idx), len(synced_idx2), len(synced_idx3))

        for i in range(5, len(joint_time)-5):
            self.pkl_data['deltaJointState'].append(self.deltaJoint_data[i])
            self.pkl_data['inHandImg'].append(self.inhandImage_data[synced_idx[i]])
            self.pkl_data['externalImg'].append(self.externalImage_data[synced_idx2[i]])
            self.pkl_data['deltaControl'].append(self.deltaControl_data[synced_idx3[i]])

        print("# joint data: ", len(self.pkl_data['deltaJointState']))
        print("# inhand image data: ", len(self.pkl_data['inHandImg']))
        print("# external image data: ", len(self.pkl_data['externalImg']))
        print("# delta control data: ", len(self.pkl_data['deltaControl']))
        pkl.dump(self.pkl_data, open(self.data_filename, 'wb'))

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--load_file_name', '-f', default='')
    parser.add_argument('--load_file_path', '-p', default='')
    parser.add_argument('--save', '-s', action='store_true')
    args = parser.parse_args()

    path_to_data = args.load_file_path
    file_time = args.load_file_name

    bag_path = path_to_data+ file_time + "/rosbags/"
    save_path = path_to_data+ file_time + "/pkls/"
    files = os.listdir(bag_path)

    for file in files:
        print("File name: ", file)
        reader = RosbagReader(bag_path, file, save_path, file_time)

        # read delta joint data
        reader.read_deltaControl()
        reader.read_deltaJoints()
        reader.read_inhandImage()
        reader.read_externalImage()
        if args.save:
            reader.save_pkl()
