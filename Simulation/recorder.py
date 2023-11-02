import os
import cv2
import numpy as np
import copy
import math
from PIL import Image

class video_recorder:
    """
    The class is desgined to capture camera images as a single video
    """

    def __init__(self, view1_size, view2_size, path, fps, num_view=2):
        self.img_size = [max(view1_size[0], view2_size[0]), max(view1_size[1], view2_size[1])]
        # print(self.img_size, vision_size, tactile_size)
        dir = os.path.dirname(os.path.abspath(path))
        os.makedirs(dir, exist_ok=True)
        if path.endswith('.avi'):
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
        elif path.endswith('.mp4'):
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        else:
            raise ValueError("Unsupported video format")
        if num_view == 2:
            self.rec = cv2.VideoWriter(path, fourcc, fps, (self.img_size[1] * 2, self.img_size[0]), True)
        else:
            self.rec = cv2.VideoWriter(path, fourcc, fps, (self.img_size[1], self.img_size[0]), True)
        self.path = path

    def capture(self, view1_image, view2_image):
        img_cap = self._align_image(view1_image, view2_image)
        self.rec.write(img_cap)

    def capture_single(self, view1_image):
        img_cap = np.zeros([self.img_size[0], self.img_size[1], 3], dtype=np.uint8)
        img_cap[:view1_image.shape[0], :view1_image.shape[1]] = (view1_image[..., :3])[..., ::-1]
        self.rec.write(img_cap)

    def release(self, new_path=None, delete=False):
        self.rec.release()
        if delete:
            os.remove(self.path)
        if new_path is not None:
            os.rename(self.path, new_path)

    def _align_image(self, img1, img2):
        assert img1.dtype == np.uint8 and img2.dtype == np.uint8
        assert max(img1.shape[0], img2.shape[0]) == self.img_size[0]
        assert max(img1.shape[1], img2.shape[1]) == self.img_size[1]

        new_img = np.zeros([self.img_size[0], self.img_size[1] * 2, 3], dtype=np.uint8)
        new_img[:img1.shape[0], :img1.shape[1]] = (img1[..., :3])[..., ::-1]
        new_img[:img2.shape[0], self.img_size[1]:self.img_size[1] + img2.shape[1]] = (img2[..., :3])[..., ::-1]
        return new_img


class Camera:
    def __init__(self, pb, cameraDistance=0.2, cameraYaw=20, cameraPitch=-20, cameraTargetPosition=[-0, 0, 0.01], cameraResolution=[320, 240]):
        self.cameraResolution = cameraResolution

        camTargetPos = cameraTargetPosition
        camDistance = cameraDistance
        upAxisIndex = 2

        yaw = cameraYaw
        pitch = cameraPitch
        roll = 0
        fov = 75
        nearPlane = 0.001
        farPlane = 2

        self.viewMatrix = pb.computeViewMatrixFromYawPitchRoll(
            camTargetPos, camDistance, yaw, pitch, roll, upAxisIndex
        )

        aspect = cameraResolution[0] / cameraResolution[1]

        self.projectionMatrix = pb.computeProjectionMatrixFOV(
            fov, aspect, nearPlane, farPlane
        )
        self.pb = pb

    def get_image(self):
        img_arr = self.pb.getCameraImage(
            self.cameraResolution[0],
            self.cameraResolution[1],
            self.viewMatrix,
            self.projectionMatrix,
            shadow=1,
            lightDirection=[1, 1, 1],
            renderer=self.pb.ER_BULLET_HARDWARE_OPENGL,
        )

        rgb = img_arr[2]  # color data RGB
        dep = img_arr[3]  # depth data
        return rgb, dep
