import os
from os import path as osp
import time
from datetime import datetime
import taichi as ti
import cv2
import numpy as np
import argparse
import subprocess


import rospy
from std_msgs.msg import Float32MultiArray, Int16MultiArray
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from Kinematics_Hand import DeltaHand


def save_rosbag(dir_path, filename="", topic='/slider /deltaControl /deltaJointState /raspicam_node/image/compressed /rgb/image_raw'):
    cmd = "rosbag record " + '-O ' + dir_path + filename + ' ' + topic
    rosbag_p = subprocess.Popen("exec " + cmd, stdout=subprocess.PIPE, shell=True)
    return rosbag_p

class DeltaServer:
    def __init__(self, enable_delta, enable_camera, save_flag, save_folder=None):

        self.enable_delta = enable_delta
        self.enable_camera = enable_camera
        self.save_flag = save_flag
        self.image = None

        self.num_motors = 12
        self.min_pos = 0.001
        self.max_pos = 0.019
        self.joint_positions = [self.min_pos] * self.num_motors

        self.num_sliders = 12
        self.min_sread = 0
        self.max_sread = 500
        self.slider_positions = [self.min_sread] * self.num_sliders

        if self.enable_delta:
            self.delta_pub = rospy.Publisher('/deltaControl', Float32MultiArray, queue_size=100)
            self.delta_sub = rospy.Subscriber('/deltaJointState', Float32MultiArray, self.jointCallback)
            print("Delta hand's joint states subscribed!")

        self.slider_sub = rospy.Subscriber("/slider", Int16MultiArray, self.sliderCallback)
        print("Slider subscribed!")

        if self.enable_camera:
            self.img_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.imgCallback)
            print("Delta hand's camera subscribed!")


        self.hand_kinematic = DeltaHand(num_finger=4, center_len=15, ee_len=6, base_len=15, leg_len=42)
        self.height = 1 *  self.hand_kinematic.hand_init_center[2] - 5.0

        if self.save_flag:
            self.save_path = save_folder
            os.makedirs(self.save_path, exist_ok=True)


    def jointCallback(self, joint_msg):
        # 0.001 - 0.019
        for i in range(self.num_motors):
            self.joint_positions[i] = joint_msg.data[i]

    def sliderCallback(self, slider_msg):
        # 0 - 500
        for i in range(self.num_sliders):
            self.slider_positions[i] = slider_msg.data[i]

    def imgCallback(self, img):
        # 480 x 640 x 3
        np_arr = np.frombuffer(img.data, np.uint8)
        self.image = cv2.cvtColor(cv2.imdecode(np_arr, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)

    def start_saving(self, file_name):
        self.rosbag_p = save_rosbag(self.save_path, file_name+".bag")

    def end_saving(self):
        self.rosbag_p.terminate()

    def reset(self):
        control_msg = Float32MultiArray()

        # reset 0/1, stop 0/1 start, 0/1, position_control 0/1, target potisions # num_motors
        control_flag = [0, 0, 0, 1]
        control_positions = [self.min_pos] * self.num_motors
        control_msg.data = control_flag + control_positions

        self.delta_pub.publish(control_msg)


    def position_move(self, targets):
        control_msg = Float32MultiArray()

        # reset 0/1, stop 0/1 start, 0/1, position_control 0/1, target potisions # num_motors
        assert len(targets) == self.num_motors
        control_flag = [0, 0, 0, 1]
        control_positions = targets.copy()
        control_msg.data = control_flag + control_positions

        self.delta_pub.publish(control_msg)

    def map_joint_to_joint(self):
        slider_range = 500.0
        delta_range = 20.0

        act = []

        for i in range(self.num_sliders):
            s2d = self.slider_positions[i] * delta_range / slider_range
            act.append(s2d)

        traj = 0.001 * np.array([act[6], act[8], act[7], act[3], act[5], act[4], act[0], act[2], act[1], act[9], act[11], act[10]])
        traj = np.clip(traj, self.min_pos, self.max_pos)
        traj = traj.tolist() # meter

        if self.enable_delta:
            control_msg = Float32MultiArray()
            assert len(traj) == self.num_motors
            control_flag = [0, 0, 0, 1]
            control_positions = traj.copy()

            control_msg.data = control_flag + control_positions
            self.delta_pub.publish(control_msg)

        viz_act = 1000.0 * np.array([[traj[6], traj[8], traj[7]],[traj[3], traj[5], traj[4]],[traj[0], traj[2], traj[1]],[traj[9], traj[11], traj[10]]])
        fk_pos = self.hand_kinematic.go_to_FK(viz_act) # 4 x 3
        return fk_pos


ti.init(arch=ti.gpu)
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--enable_delta', '-d', action='store_true')
    parser.add_argument('--enable_camera', '-c', action='store_true')
    parser.add_argument('--save_rosbag', '-s', action='store_true')
    parser.add_argument('--save_path', '-p', default='')
    args = parser.parse_args()

    rospy.init_node('delta_hand_server', anonymous=True)
    rate = rospy.Rate(200) # 200hz

    save_folder = None
    if args.save_rosbag:
        now = datetime.now()
        save_folder = args.save_path + now.strftime('%d_%m_%Y') + "/rosbags/"
        save_file_name = input("Enter the file name to save:")

    delta_hand_teleop = DeltaServer(args.enable_delta, args.enable_camera, args.save_rosbag, save_folder)
    gui = ti.GUI("Finger pos visualizer", (640, 480))

    delta_hand_teleop.reset()
    time.sleep(1.0)


    viz_line_X = np.array([[0.01,0.5], [0.5, 0.01]])
    viz_line_Y = np.array([[0.99, 0.5],[0.5, 0.99]])

    if args.save_rosbag:
        total_teleop_time = 5000
        time_cnt = 0
        print("Starting slider teleoperation in 3 seconds...")
        time.sleep(3.0)
        delta_hand_teleop.start_saving(save_file_name)
    else:
        total_teleop_time = 500000
        time_cnt = 0
        print("Starting slider teleoperation in 1 seconds...")
        time.sleep(1.0)


    while time_cnt < total_teleop_time and gui.running:
        for e in gui.get_events(gui.PRESS):
            if e.key == gui.ESCAPE:
                gui.running = False

        fk_pos = delta_hand_teleop.map_joint_to_joint()

        draw_fk_pos = np.array(fk_pos)[:,0:2]  * 0.01 + 0.5
        if delta_hand_teleop.enable_camera:
            gui.set_image(cv2.rotate(delta_hand_teleop.image, cv2.ROTATE_90_COUNTERCLOCKWISE))

        gui.lines(begin=viz_line_X, end=viz_line_Y, radius=2, color=0x068587)
        gui.circles(draw_fk_pos, radius=20, color=0x9403fc) # yellow
        gui.show()

        time_cnt += 1

    if args.save_rosbag:
        delta_hand_teleop.end_saving()
    delta_hand_teleop.reset()
    time.sleep(1.0)
