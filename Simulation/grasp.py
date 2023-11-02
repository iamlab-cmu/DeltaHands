# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import argparse
import datetime
import os
import time
import numpy as np
import pybullet as pb
import pybullet_data
import matplotlib.pyplot as plt
from collections import defaultdict
import cv2
from robot import Robot
import recorder
from config import ObjConfig

def get_forces(pb, bodyA=None, bodyB=None, linkIndexA=None, linkIndexB=None):
    """
    get contact forces
    :return: normal force, lateral force
    """
    kwargs = {
        "bodyA": bodyA,
        "bodyB": bodyB,
        "linkIndexA": linkIndexA,
        "linkIndexB": linkIndexB,
    }
    kwargs = {k: v for k, v in kwargs.items() if v is not None}

    pts = pb.getContactPoints(**kwargs)

    totalNormalForce = 0
    totalLateralFrictionForce = [0, 0, 0]

    for pt in pts:
        totalNormalForce += pt[9]

        totalLateralFrictionForce[0] += pt[11][0] * pt[10] + pt[13][0] * pt[12]
        totalLateralFrictionForce[1] += pt[11][1] * pt[10] + pt[13][1] * pt[12]
        totalLateralFrictionForce[2] += pt[11][2] * pt[10] + pt[13][2] * pt[12]

    return totalNormalForce, totalLateralFrictionForce

if __name__ == "__main__":

    # Initialize World
    print("Initializing...")

    physicsClient = pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    # pb.setRealTimeSimulation(0)
    # pb.setTimeStep(1 / 1000)
    pb.setTimeStep(1 / 240)
    pb.setGravity(0, 0, -9.81)  # Major Tom to planet Earth

    pb.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=80, cameraPitch=-20, cameraTargetPosition=[0.5, 0, 0.01], physicsClientId=physicsClient)
    planeId = pb.loadURDF("plane.urdf")  # Create plane

    # robotURDF = "urdfs/ur5e-deltaL.urdf"
    robotURDF = "urdfs/ur5e-delta5.urdf"
    robotID = pb.loadURDF(robotURDF, useFixedBase=True)
    rob = Robot(robotID)

    obj = ObjConfig("073-c_lego_duplo", [0.50, 0.02, 0.0], 0.15, 0.0055, 0.25*np.pi, [2.32492204,3+10.9502145,3+11.80304592,3+10.03108074,3+10.97149808])
    obj_name = obj.name
    objURDF = "urdfs/objects/" + obj_name + "/model.urdf"
    objStartPos = obj.loc
    objStartOrientation = pb.getQuaternionFromEuler([0.0, 0.0, obj.angle])
    objID = pb.loadURDF(objURDF, objStartPos, objStartOrientation)
    pb.resetBasePositionAndOrientation(objID, objStartPos, objStartOrientation)
    for _ in range(20):
        pb.stepSimulation()

    start_time = time.time()
    init_pos = [0.5, 0.0, obj.height]
    init_rot = 0
    init_linear = 0.0
    init_rotary = 0.0

    cur_pos = init_pos.copy()
    dz = obj.dz

    grasp_steps = 20

    # linear motors
    # opening
    omotor1_range = np.linspace(0.005, 0.015, grasp_steps, endpoint=True)
    omotor2_range = np.linspace(0.015, 0.005, grasp_steps, endpoint=True)
    omotor3_range = np.linspace(0.015, 0.005, grasp_steps, endpoint=True)
    # closing
    cmotor1_range = np.linspace(0.015, 0.005, grasp_steps, endpoint=True)
    cmotor2_range = np.linspace(0.0, 0.01, grasp_steps, endpoint=True)
    cmotor3_range = np.linspace(0.0, 0.018, grasp_steps, endpoint=True)
    grasp_idx = 0
    open_idx = 0

    cmotor1_range = np.linspace(0.015, obj.act[0] * 1e-3, grasp_steps, endpoint=True)
    cmotor2_range = np.linspace(0.0, obj.act[1] * 1e-3, grasp_steps, endpoint=True)
    cmotor3_range = np.linspace(0.0, obj.act[2] * 1e-3, grasp_steps, endpoint=True)
    cmotor4_range = np.linspace(0.0, obj.act[3] * 1e-3, grasp_steps, endpoint=True)
    cmotor5_range = np.linspace(0.0, obj.act[4] * 1e-3, grasp_steps, endpoint=True)

    force_lists = [[],[],[],[]]


    ## record the grasp
    record_video = True
    if record_video:
        video_name = obj_name+"_grasp.mp4"
        cam1 = recorder.Camera(pb, cameraDistance=0.5, cameraYaw=80, cameraPitch=-20, cameraTargetPosition=[0.5, 0, 0.01], cameraResolution=[640*2, 480*2])
        view1_img, _ = cam1.get_image()
        save_dir = os.path.join("..", 'results', 'videos')
        os.makedirs(save_dir, exist_ok=True)
        video_path = os.path.join(save_dir, video_name)
        rec = recorder.video_recorder(view1_img.shape, view1_img.shape, path=video_path, fps=5, num_view=1)

    t = 0
    # pick_and_place()
    if t == 0:
        # initialization
        rob.init_robot(arm_init_pos=init_pos, arm_init_rot=init_rot, delta_linear_init=init_linear, delta_rotary_init=init_rotary)
        motor_actuations = [0.01]

        # linear
        for i in range(4):
            motor_actuations.append(0.0)
        rob.operate_delta(motor_actuations, wait=True)
        p = input("pause")
    for t in range(10):
        # Dropping
        cur_pos[2] -= dz
        rob.operate_arm(cur_pos, rot=None, wait=True)

        if record_video:
            view1_img, _ = cam1.get_image()
            rec.capture_single(view1_img.copy())
    for t in range(20):
        # Grasping
        motor_actuations = [cmotor1_range[grasp_idx], cmotor2_range[grasp_idx], cmotor3_range[grasp_idx], cmotor4_range[grasp_idx], cmotor5_range[grasp_idx]]
        rob.operate_delta(motor_actuations, wait=True)
        grasp_idx += 1

        FnA, FfA = get_forces(pb, rob.sphereA, objID, -1,-1)
        FnB, FfB = get_forces(pb, rob.sphereB, objID, -1,-1)
        FnC, FfC = get_forces(pb, rob.sphereC, objID, -1,-1)
        FnD, FfD = get_forces(pb, rob.sphereD, objID, -1,-1)
        print("Contact forces A/B/C/D: ", FnA, FnB, FnC, FnD)
        force_lists[0].append(FnA)
        force_lists[1].append(FnB)
        force_lists[2].append(FnC)
        force_lists[3].append(FnD)

        real_ee_ori = rob.get_delta_ee_ori()
        gt_ori = [[np.pi, 0, 0], [np.pi, 0, -np.pi/2], [np.pi, 0, np.pi], [np.pi, 0, np.pi/2]]
        ori_err = []
        for i in range(4):
            ee_err = []
            for j in range(3):
                ee_err += [np.abs(gt_ori[i][j] - real_ee_ori[i][j])]
            ori_err.append(ee_err)
        print("Delta ee's ori err: ", ori_err)
        real_state = rob.get_delta_motor_state()
        motor_err = []
        for i in range(4):
            motor_err += [np.abs(motor_actuations[i] - real_state[i])]
        print("Delta motor's err: ", motor_err)
        # p = input("Pause")
        normal_forces = [FnA, FnB, FnC, FnD]

        if np.max(normal_forces) > 3.0:
            break
        if record_video:
            view1_img, _ = cam1.get_image()
            rec.capture_single(view1_img.copy())

    for t in range(10):
        # Lifting
        cur_pos[2] += dz
        rob.operate_arm(cur_pos, rot=None, wait=True)
        if record_video:
            view1_img, _ = cam1.get_image()
            rec.capture_single(view1_img.copy())
    print("Finished!")
    pb.disconnect()  # Close PyBullet
    if record_video:
        rec.release()

    ## draw force curves
    fig = plt.figure()
    X = np.arange(len(force_lists[0]))
    plt.plot(X, force_lists[0], label = "finger 0")
    plt.plot(X, force_lists[1], label = "finger 1")
    plt.plot(X, force_lists[2], label = "finger 2")
    plt.plot(X, force_lists[3], label = "finger 3")
    plt.legend()
    plt.show()
