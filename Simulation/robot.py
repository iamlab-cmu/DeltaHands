# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
import os
import numpy as np
import pybullet as pb


class Robot:
    def __init__(self, robotID):
        self.robotID = robotID

        # Get link/joint ID for arm
        self.armNames = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.armJoints = self.get_id_by_name(self.armNames)
        # question mark here? do we need it for IK?
        self.armControlID = self.get_control_id_by_name(self.armNames)

        # delta
        self.jointNameToID = {}
        self.linkNameToID = {}
        self.revoluteID = []
        self.prismaticID = []
        self.motorName = []

        for j in range(pb.getNumJoints(self.robotID)):
            info = pb.getJointInfo(self.robotID, j)
            # print(info)
            jointID = info[0]
            jointName = info[1].decode('UTF-8')
            jointType = info[2]
            if jointName[0:12] == "motor_joint_":
                self.motorName.append(jointName)
            if jointName in self.armNames:
                continue
            if (jointType == pb.JOINT_PRISMATIC):
                print("prismatic: ", jointName)
                self.jointNameToID[jointName] = jointID
                self.linkNameToID[info[12].decode('UTF-8')] = jointID
                self.prismaticID.append(jointID)
            if (jointType == pb.JOINT_REVOLUTE):
                print("revolute: ", jointName)
                self.jointNameToID[jointName] = jointID
                self.linkNameToID[info[12].decode('UTF-8')] = jointID
                self.revoluteID.append(jointID)

        self.motorJoints = self.get_id_by_name(self.motorName)

        # set the universal joints
        motorID = []
        for motor_name in self.motorName:
            motorID.append(self.jointNameToID[motor_name])
        for i in self.revoluteID:
            if i in motorID:
                pass
            else:
                # set friction of the other joint to 0.001
                pb.setJointMotorControl2(self.robotID, i, controlMode=pb.VELOCITY_CONTROL, targetVelocity=0, force=0.001)

        # Get ID for end effector
        self.eeName = ["ee_link"]
        self.eefID = self.get_id_by_name(self.eeName)[0]


        self.armHome = [-0.26730250468455913, -1.571434616558095, 1.7978336633640315,
                        -1.797156403649463, -1.570812383908381, -0.26729946203972366]
        self.armMotorMaxForce = 200

        self.linearMotorHome = 0.0
        self.rotaryMotorHome = 0.0
        self.linearMotorMaxForce = 45
        self.rotaryMotorMaxForce = 45
        self.linearMotorIdx = []
        self.rotaryMotorIdx = []
        self.delta_max_forces = [45 for _ in range(len(self.motorJoints))]

        # ur5e arm's current pose
        self.pos = [0.581, 0.002, 0.445]
        self.ori = [0, np.pi / 2, 0]
        self.rot = 0

        # deltas' current pose
        self.motor_joint_pos = [0.0 for _ in range(len(self.motorJoints))]

        self.tol = 1e-6
        self.max_iter = 30

        self.init_robot()

        # create the end effector platform
        self.create_delta_ee()

    def get_id_by_name(self, names):
        """
        get joint/link ID by name
        """
        nbJoint = pb.getNumJoints(self.robotID)
        jointNames = {}
        for i in range(nbJoint):
            name = pb.getJointInfo(self.robotID, i)[1].decode()
            jointNames[name] = i

        return [jointNames[name] for name in names]

    def get_control_id_by_name(self, names):
        """
        get joint/link ID by name
        """
        nbJoint = pb.getNumJoints(self.robotID)
        jointNames = {}
        ctlID = 0
        for i in range(nbJoint):
            jointInfo = pb.getJointInfo(self.robotID, i)
            name = jointInfo[1].decode("utf-8")

            # skip fixed joint
            if jointInfo[2] == 4:
                continue

            # skip base joint
            if jointInfo[-1] == -1:
                continue
            jointNames[name] = ctlID
            ctlID += 1

        return [jointNames[name] for name in names]

    def create_delta_ee(self):
        self.robotScale = 1
        self.robotPos, _ = self.get_ee_pose()
        orientation = [np.pi, 0, np.pi]
        ball_radius = 0.006 * self.robotScale
        ball_height = 0.005 * self.robotScale
        ballpos = [self.robotPos[0], self.robotPos[1], self.robotPos[2] - 0.06 * self.robotScale]
        visualShapeId = -1

        offset = [0.0, -1*(0.02+0.01)]
        ballposA = [self.robotPos[0]+offset[0], self.robotPos[1]+offset[1], self.robotPos[2] - 0.08 * self.robotScale]
        self.sphereA = pb.loadURDF(os.path.abspath(os.path.dirname(__file__)) + '/urdfs/fingertip/fingertip.urdf',
                           ballposA,
                           pb.getQuaternionFromEuler([-np.pi, 0, 0]), # [0, 0, 0]
                           globalScaling=self.robotScale)

        offset = [-1*(0.02+0.01), 0.0]
        ballposB = [self.robotPos[0]+offset[0], self.robotPos[1]+offset[1], self.robotPos[2] - 0.08 * self.robotScale]
        self.sphereB = pb.loadURDF(os.path.abspath(os.path.dirname(__file__)) + '/urdfs/fingertip/fingertip.urdf',
                           ballposB,
                           pb.getQuaternionFromEuler([-np.pi, 0, -np.pi/2]), # [0, 0, 0]
                           globalScaling=self.robotScale)

        offset = [0.0, 1*(0.02+0.01)]
        ballposC = [self.robotPos[0]+offset[0], self.robotPos[1]+offset[1], self.robotPos[2] - 0.08 * self.robotScale]
        self.sphereC = pb.loadURDF(os.path.abspath(os.path.dirname(__file__)) + '/urdfs/fingertip/fingertip.urdf',
                           ballposC,
                           pb.getQuaternionFromEuler([-np.pi, 0, np.pi]), # [0, 0, 0]
                           globalScaling=self.robotScale)

        offset = [1*(0.02+0.01), 0.0]
        ballposD = [self.robotPos[0]+offset[0], self.robotPos[1]+offset[1], self.robotPos[2] - 0.08 * self.robotScale]
        self.sphereD = pb.loadURDF(os.path.abspath(os.path.dirname(__file__)) + '/urdfs/fingertip/fingertip.urdf',
                           ballposD,
                           pb.getQuaternionFromEuler([-np.pi, 0, np.pi/2]), # [0, 0, 0]
                           globalScaling=self.robotScale)

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

        def get_a_set(center, top, topl, topr, right_angle, left_angle):
            bottom_right = rotate(center, top, right_angle); bottom_rightl = rotate(center, topl, right_angle); bottom_rightr = rotate(center, topr, right_angle)
            bottom_left = rotate(center, top, left_angle); bottom_leftl = rotate(center, topl, left_angle); bottom_leftr = rotate(center, topr, left_angle)
            return top, topl, topr, bottom_right, bottom_rightl, bottom_rightr, bottom_left, bottom_leftl, bottom_leftr


        all_spheres = [self.sphereA, self.sphereB, self.sphereC, self.sphereD]
        all_set_angle = [180, 180, 180, 180]
        all_constraint_position = []

        for i in range(len(all_spheres)):
            # set 1
            center = [0.0, 0.0]
            topP = [0.0, 0.006]; topPl = [-0.005, 0.006]; topPr = [0.005, 0.006]
            set_angle = all_set_angle[i]
            topP = rotate(center, topP, set_angle); topPl = rotate(center, topPl, set_angle); topPr = rotate(center, topPr, set_angle);

            right_angle = -120.0
            left_angle = 120.0
            topP, topPl, topPr, bottom_right, bottom_rightl, bottom_rightr, bottom_left, bottom_leftl, bottom_leftr = get_a_set(center, topP, topPl, topPr, right_angle, left_angle)

            # from top left leg, clockwise circle
            constraint_position = [[[topPl[0], topPl[1], 0.0], [topPr[0], topPr[1], 0.0]],
                                    [[bottom_rightl[0], bottom_rightl[1], 0.0], [bottom_rightr[0], bottom_rightr[1], 0.0]],
                                    [[bottom_leftl[0], bottom_leftl[1], 0.0], [bottom_leftr[0], bottom_leftr[1], 0.0]]]
            all_constraint_position.append(constraint_position)

        # # delta L
        # A_parent_links = ['parallel_1_', 'parallel_5_', 'parallel_6_']
        # B_parent_links = ['parallel_2_', 'parallel_7_', 'parallel_8_']
        # C_parent_links = ['parallel_3_', 'parallel_9_', 'parallel_10_']
        # D_parent_links = ['parallel_4_', 'parallel_11_', 'parallel_12_']

        # delta5
        A_parent_links = ['parallel_11_', 'parallel_22_', 'parallel_51_']
        B_parent_links = ['parallel_12_', 'parallel_32_', 'parallel_21_']
        C_parent_links = ['parallel_13_', 'parallel_42_', 'parallel_31_']
        D_parent_links = ['parallel_14_', 'parallel_52_', 'parallel_41_']
        all_parent_links = [A_parent_links, B_parent_links, C_parent_links, D_parent_links]

        for i in range(len(all_parent_links)):
            parent_links = all_parent_links[i]
            constraint_position = all_constraint_position[i]
            sphere = all_spheres[i]
            for j in range(len(parent_links)):
                print(constraint_position[j])
                pb.createConstraint(parentBodyUniqueId=self.robotID,
                                   parentLinkIndex=self.linkNameToID[parent_links[j]+'leg1'],
                                   childBodyUniqueId=sphere,
                                   childLinkIndex=-1,
                                   jointType=pb.JOINT_POINT2POINT,
                                   jointAxis=[0, 0, 0],
                                   parentFramePosition=[0, 0, 0.04],
                                   childFramePosition=constraint_position[j][0])
                pb.createConstraint(parentBodyUniqueId=self.robotID,
                                   parentLinkIndex=self.linkNameToID[parent_links[j]+'leg2'],
                                   childBodyUniqueId=sphere,
                                   childLinkIndex=-1,
                                   jointType=pb.JOINT_POINT2POINT,
                                   jointAxis=[0, 0, 0],
                                   parentFramePosition=[0, 0, 0.04],
                                   childFramePosition=constraint_position[j][1])

    def reset_robot(self):
        for j in range(len(self.armJoints)):
            pb.resetJointState(self.robotID, self.armJoints[j], self.armHome[j])
        self.delta_max_forces = []
        for j, motor_name in enumerate(self.motorName):
            if self.jointNameToID[motor_name] in self.prismaticID:
                pb.resetJointState(self.robotID, self.jointNameToID[motor_name], self.linearMotorHome)
                self.delta_max_forces.append(self.linearMotorMaxForce)
                self.linearMotorIdx.append(j)
            if self.jointNameToID[motor_name] in self.revoluteID:
                pb.resetJointState(self.robotID, self.jointNameToID[motor_name], self.rotaryMotorHome)
                self.delta_max_forces.append(self.rotaryMotorMaxForce)
                self.rotaryMotorIdx.append(j)

    def init_robot(self, arm_init_pos=None, arm_init_rot=None, delta_linear_init=None, delta_rotary_init=None):
        self.reset_robot()

        if arm_init_pos is not None:
            self.pos = arm_init_pos
        if arm_init_rot is not None:
            self.rot = arm_init_rot
        self.operate_arm(self.pos, self.rot, wait=True)

        if delta_linear_init is not None:
            for idx in self.linearMotorIdx:
                self.motor_joint_pos[idx] = delta_linear_init
        if delta_rotary_init is not None:
            for idx in self.rotaryMotorIdx:
                self.motor_joint_pos[idx] = delta_rotary_init
        self.operate_delta(self.motor_joint_pos, wait=True)

    # Get the position and orientation of the UR5 end-effector
    def get_ee_pose(self):
        res = pb.getLinkState(self.robotID, self.eefID)
        world_positions = res[0]
        world_orientations = res[1]
        return world_positions, world_orientations

    # Get the joint angles for ur5e
    def get_arm_state(self):
        all_states = [_[0] for _ in pb.getJointStates(self.robotID, self.armJoints)]
        return all_states

    # Get the joint angles for delta robots
    def get_delta_motor_state(self):
        delta_states = [_[0] for _ in pb.getJointStates(self.robotID, self.motorJoints)]
        return delta_states

    def get_delta_ee_pos(self):
        ee_pos1 = np.asarray(pb.getBasePositionAndOrientation(self.sphereA)[0])
        ee_pos2 = np.asarray(pb.getBasePositionAndOrientation(self.sphereB)[0])
        ee_pos3 = np.asarray(pb.getBasePositionAndOrientation(self.sphereC)[0])
        ee_pos4 = np.asarray(pb.getBasePositionAndOrientation(self.sphereD)[0])
        return [ee_pos1, ee_pos2, ee_pos3, ee_pos4]

    def get_delta_ee_ori(self):
        ee_ori1 = np.asarray(pb.getEulerFromQuaternion(pb.getBasePositionAndOrientation(self.sphereA)[1]))
        ee_ori2 = np.asarray(pb.getEulerFromQuaternion(pb.getBasePositionAndOrientation(self.sphereB)[1]))
        ee_ori3 = np.asarray(pb.getEulerFromQuaternion(pb.getBasePositionAndOrientation(self.sphereC)[1]))
        ee_ori4 = np.asarray(pb.getEulerFromQuaternion(pb.getBasePositionAndOrientation(self.sphereD)[1]))
        return [ee_ori1, ee_ori2, ee_ori3, ee_ori4]

    def get_rotation(self):
        _, ori_q = self.get_ee_pose()
        ori = pb.getEulerFromQuaternion(ori_q)
        rot = ori[-1]
        return rot

    # position control of the ur5e arm (deltas are static)
    def operate_arm(self, pos, rot=None,wait=False):
        # only rotate the last joint before EE
        if rot is None:
            ori = self.ori.copy()
        else:
            ori = self.ori.copy()
            ori[-1] = rot
        ori_q = pb.getQuaternionFromEuler(ori)

        joint_pose = pb.calculateInverseKinematics(self.robotID, self.eefID, pos, ori_q)#[self.armControlID]
        arm_joint_pose = list(joint_pose)[0:6]
        arm_max_forces = [self.armMotorMaxForce for _ in range(len(arm_joint_pose))]

        delta_joint_pose = self.get_delta_motor_state()
        delta_max_forces = self.delta_max_forces

        # Select the relavant joints for arm
        jointPose = np.array(arm_joint_pose+delta_joint_pose)
        maxForces = np.array(arm_max_forces+delta_max_forces)

        pb.setJointMotorControlArray(
            self.robotID,
            tuple(self.armJoints+self.motorJoints),
            pb.POSITION_CONTROL,
            targetPositions=jointPose,
            forces=maxForces,
        )

        self.pos = pos
        if ori is not None:
            self.ori = ori

        if wait:
            last_err = 1e6
            cnt = 0
            while (cnt < self.max_iter):
                cnt += 1
                pb.stepSimulation()
                ee_pose = self.get_ee_pose()
                err = (
                        np.sum(np.abs(np.array(ee_pose[0]) - pos))
                        + np.sum(np.abs(np.array(ee_pose[1]) - ori_q))
                )
                diff_err = last_err - err
                last_err = err

                if np.abs(diff_err) < self.tol:
                    break
            if cnt == self.max_iter:
                # print(self.get_delta_motor_state())
                print("Unstable simulation step!!!", np.abs(diff_err), np.abs(err))

    # position control of delta actuators (robot arm is static)
    def operate_delta(self, motor_actuations, wait=False):

        arm_joint_pose = self.get_arm_state()
        arm_max_forces = [self.armMotorMaxForce for _ in range(len(arm_joint_pose))]

        delta_joint_pose = motor_actuations
        delta_max_forces = self.delta_max_forces

        # Select the relavant joints for arm
        jointPose = np.array(arm_joint_pose + delta_joint_pose)
        maxForces = np.array(arm_max_forces+delta_max_forces)

        pb.setJointMotorControlArray(
            self.robotID,
            tuple(self.armJoints+self.motorJoints),
            pb.POSITION_CONTROL,
            targetPositions=jointPose,
            forces=maxForces,
        )

        if wait:
            last_err = 1e6
            cnt = 0
            while (cnt < self.max_iter):
                cnt += 1
                pb.stepSimulation()
                cur_state = self.get_delta_motor_state()
                err = np.sum(np.abs(np.array(cur_state) - motor_actuations))
                diff_err = last_err - err
                last_err = err

                if np.abs(diff_err) < self.tol:
                    break
            if cnt == self.max_iter:
                print("Unstable simulation step!!!", np.abs(diff_err), np.abs(err))
