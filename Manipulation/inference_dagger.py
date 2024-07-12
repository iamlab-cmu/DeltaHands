import cv2
import os
import sys
from typing import Tuple, Sequence, Dict, Union, Optional, Callable
import numpy as np
import math
import taichi as ti
import pickle as pkl
import torch
import torch.nn as nn
import torchvision
import collections
import argparse
import subprocess

from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from diffusers.training_utils import EMAModel
from diffusers.optimization import get_scheduler
from Kinematics_Hand import DeltaHand

from dataset.data_utils import *
from model.noise_pred_net import *
from model.visual_encoder import *
from config import *
from delta_env import *

def save_rosbag(dir_path, filename="", topic='/deltaControl /deltaJointState /raspicam_node/image/compressed /rgb/image_raw'):
    cmd = "rosbag record " + '-O ' + dir_path + filename + ' ' + topic
    rosbag_p = subprocess.Popen("exec " + cmd, stdout=subprocess.PIPE, shell=True)
    return rosbag_p

class DiffPolicyInfer:
    def __init__(self, file_name, save_flag, record_flag, ckpt_path, save_path):

        self.num_motors = 12
        self.min_pos = 0.001
        self.max_pos = 0.019
        self.joint_positions = [self.min_pos] * self.num_motors

        self.num_sliders = 12
        self.min_sread = 0
        self.max_sread = 500
        self.slider_positions = [self.min_sread] * self.num_sliders


        self.slider_sub = rospy.Subscriber("/slider", Int16MultiArray, self.sliderCallback)
        print("Slider subscribed!")

        vision_encoder = get_resnet('resnet18')
        vision_encoder = replace_bn_with_gn(vision_encoder)
        noise_pred_net = ConditionalUnet1D(
            input_dim=action_dim,
            global_cond_dim=obs_dim*obs_horizon
        )
        nets = nn.ModuleDict({
            'vision_encoder': vision_encoder,
            'noise_pred_net': noise_pred_net
        })

        self.num_diffusion_iters = 80 #100
        self.noise_scheduler = DDPMScheduler(
            num_train_timesteps=self.num_diffusion_iters,
            beta_schedule='squaredcos_cap_v2',
            clip_sample=True,
            prediction_type='epsilon'
        )

        # device transfer
        self.device = torch.device('cuda')
        _ = nets.to(self.device)

        self.ckpt_path = ckpt_path + file_name + ".pt"
        if not os.path.isfile(self.ckpt_path):
          print("Err at loading trained model!")

        state_dict = torch.load(self.ckpt_path, map_location='cuda')
        self.ema_nets = nets
        self.ema_nets.load_state_dict(state_dict['model_state_dict'])
        print('Pretrained weights loaded.')

        self.max_steps = 200
        self.cur_img = None
        self.cur_action = None

        ## init model
        self.hand_kinematic = DeltaHand(num_finger=4, center_len=15, ee_len=6, base_len=20, leg_len=42)
        self.delta_env = DeltaEnv(enable_delta=True, enable_camera=True, reset_delta=True)

        init_action = [0.0] * self.delta_env.num_motors
        init_obs = self.delta_env.step(init_action)
        self.cur_img = init_obs['oimage']
        self.cur_action = init_action

        self.obs_deque = collections.deque([init_obs] * obs_horizon, maxlen=obs_horizon)

        self.save_flag = save_flag
        self.record_flag = record_flag
        if self.save_flag or self.record_flag:
            self.save_file_name = input("Enter the file name to save data: ")
            self.save_path = save_path

        self.in_record = False
        if self.record_flag:
            self.start_saving(self.save_file_name)


    def sliderCallback(self, slider_msg):
        # 0 - 500
        for i in range(self.num_sliders):
            self.slider_positions[i] = slider_msg.data[i]

    def finish(self):
        if self.save_flag or self.record_flag:
            self.end_saving()
        self.delta_env.reset()
        time.sleep(1.0)

    def start_saving(self, file_name):
        if self.in_record == False:
            self.rosbag_p = save_rosbag(self.save_path, file_name+".bag")
            self.in_record = True

    def end_saving(self):
        if self.in_record == True:
            self.rosbag_p.terminate()
            self.in_record = False

    def step(self):

        B = 1
        # stack the last obs_horizon number of observations
        images = np.stack([x['image'] for x in self.obs_deque]) ## (obs_horizon,3,160,120)
        agent_poses = np.stack([x['agent_pos'] for x in self.obs_deque]) ## (obs_horizon, lowdim_obs_dim)

        # normalize observation
        nagent_poses = normalize_data(agent_poses, stats=stats)
        # images are already normalized to [0,1]
        images = np.moveaxis(images, -1,1)
        nimages = normalize_images(images)

        # device transfer
        nimages = nimages.to(self.device, dtype=torch.float32)
        # (2,3,96,96)
        nagent_poses = torch.from_numpy(nagent_poses).to(self.device, dtype=torch.float32)
        # (2,2)

        # infer action
        with torch.no_grad():
            # get image features
            image_features = self.ema_nets['vision_encoder'](nimages)
            # (2,512)

            # concat with low-dim observations
            obs_features = torch.cat([image_features, nagent_poses], dim=-1)

            # reshape observation to (B,obs_horizon*obs_dim)
            obs_cond = obs_features.unsqueeze(0).flatten(start_dim=1)

            # initialize action from Guassian noise
            noisy_action = torch.randn(
                (B, pred_horizon, action_dim), device=self.device)
            naction = noisy_action

            # init scheduler
            self.noise_scheduler.set_timesteps(self.num_diffusion_iters)

            for k in self.noise_scheduler.timesteps:
                # predict noise
                noise_pred = self.ema_nets['noise_pred_net'](
                    sample=naction,
                    timestep=k,
                    global_cond=obs_cond
                )

                # inverse diffusion step (remove noise)
                naction = self.noise_scheduler.step(
                    model_output=noise_pred,
                    timestep=k,
                    sample=naction
                ).prev_sample

        # unnormalize action
        naction = naction.detach().to('cpu').numpy()
        # (B, pred_horizon, action_dim)
        naction = naction[0]
        action_pred = unnormalize_data(naction, stats=stats)

        # only take action_horizon number of actions
        start = obs_horizon - 1
        end = start + action_horizon
        actions = action_pred[start:end,:]

        return actions

    def teleop(self):
        slider_range = 500.0
        delta_range = 20.0

        act = []
        for i in range(self.num_sliders):
            s2d = self.slider_positions[i] * delta_range / slider_range
            act.append(s2d)

        action = 0.001 * np.array([act[6], act[8], act[7], act[3], act[5], act[4], act[0], act[2], act[1], act[9], act[11], act[10]])
        return [action]

    def excute(self, action):
        # stepping env
        obs = self.delta_env.step(action)
        # save observations
        self.obs_deque.append(obs)
        self.cur_img = obs['oimage']
        self.cur_action = action

ti.init(arch=ti.gpu)
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--file_name', '-f', default='')
    parser.add_argument('--save_flag', '-s', action='store_true')
    parser.add_argument('--record_flag', '-r', action='store_true')
    parser.add_argument('--ckpt_path', '-cp', default='')
    parser.add_argument('--save_path', '-sp', default='')
    args = parser.parse_args()

    rospy.init_node("delta_hand_auto")

    ckpt_path = args.ckpt_path
    save_path = args.save_path + args.file_name + "/"
    os.makedirs(save_path, exist_ok=True)

    runner = DiffPolicyInfer(args.file_name, args.save_flag, args.record_flag, ckpt_path, save_path)

    gui = ti.GUI("Observations", (640, 480))
    colors=[0x0000ff, 0x6a82fb, 0x6897bb, 0x7fa6ee, 0xa8b2fc, 0xaec6f4, 0xb0e0e6, 0xc6e2ff]

    mode = 1 # 1 is auto; 0 is teleop
    total_teleop_time = 5000
    time_cnt = 0
    pre_teleop_pos = None

    while time_cnt < total_teleop_time and gui.running:
        for e in gui.get_events(gui.PRESS):
            if e.key == gui.ESCAPE:
                gui.running = False
            elif e.key == "p":
                mode = 2
                pre_teleop_pos = runner.cur_action.copy()
                print("Current pos: ", pre_teleop_pos)

            elif e.key == "t":
                mode = 0
                print("Switching to teleop..")

                if args.save_flag:
                    runner.start_saving(runner.save_file_name)
            elif e.key == "a":
                mode = 1
                print("Switching to auto...")
                if args.save_flag:
                    runner.end_saving()
        if gui.running == False:
            break
        # print("# step: ", time_cnt)

        actions = []
        if mode == 1:
            actions = runner.step()
        elif mode == 0:
            actions = runner.teleop()
        elif mode == 2:
            viz_actions = runner.teleop()

        if mode != 2:
            # execute action_horizon number of steps
            for i in range(len(actions)):
                runner.excute(actions[i])

                # ### visualize the actions
                gui.set_image(cv2.cvtColor(cv2.rotate(runner.cur_img, cv2.ROTATE_90_COUNTERCLOCKWISE), cv2.COLOR_BGR2RGB))

                traj = runner.cur_action
                viz_act = 1000.0 * np.array([[traj[6], traj[8], traj[7]],[traj[3], traj[5], traj[4]],[traj[0], traj[2], traj[1]],[traj[9], traj[11], traj[10]]])
                fk_pos = runner.hand_kinematic.go_to_FK(viz_act) # 4 x 3
                draw_fk_pos = np.array(fk_pos)[:,0:2]  * 0.01 + 0.5
                gui.circles(draw_fk_pos, radius=20, color=0x9403fc)
                gui.show()

            ###
            time_cnt += 1
            print("# steps ", time_cnt)
        if mode == 2:
            # ### visualize the actions
            gui.set_image(cv2.cvtColor(cv2.rotate(runner.cur_img, cv2.ROTATE_90_COUNTERCLOCKWISE), cv2.COLOR_BGR2RGB))

            traj = pre_teleop_pos
            viz_act = 1000.0 * np.array([[traj[6], traj[8], traj[7]],[traj[3], traj[5], traj[4]],[traj[0], traj[2], traj[1]],[traj[9], traj[11], traj[10]]])
            desired_fk_pos = np.array(runner.hand_kinematic.go_to_FK(viz_act)) # 4 x 3
            desired_draw_fk_pos = desired_fk_pos[:,0:2]  * 0.01 + 0.5
            gui.circles(desired_draw_fk_pos, radius=20, color=0x9403fc)

            traj = viz_actions[0]
            viz_act = 1000.0 * np.array([[traj[6], traj[8], traj[7]],[traj[3], traj[5], traj[4]],[traj[0], traj[2], traj[1]],[traj[9], traj[11], traj[10]]])
            actual_fk_pos = np.array(runner.hand_kinematic.go_to_FK(viz_act)) # 4 x 3
            actual_draw_fk_pos = actual_fk_pos[:,0:2]  * 0.01 + 0.5
            gui.circles(actual_draw_fk_pos, radius=20, color=0xFFAA33)
            gui.show()

            diff_fk_pos = desired_fk_pos[:,2] - actual_fk_pos[:,2]

            string_diff_fk_pos = str(diff_fk_pos)
            sys.stdout.write(string_diff_fk_pos + '\r')
            sys.stdout.flush()

    runner.finish()
