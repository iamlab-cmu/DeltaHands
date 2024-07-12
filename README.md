# Learning for Dexterous Manipulation with DeltaHands
This repository is the code base of the DeltaHands development. The research outputs include [DELTAHANDS](https://arxiv.org/abs/2310.05266) published on RA-L and [Tilde](https://arxiv.org/abs/2405.18804) published on RSS24. Please read below for more details.

## DELTAHANDS: A Synergistic Dexterous Hand Framework Based on Delta Robots
DELTAHANDS is a synergistic dexterous hand framework that utilizes soft Delta robots. For more information, please see our [paper](https://arxiv.org/abs/2310.05266) or the [website](https://sites.google.com/view/deltahands/).

### Control
[Arduino code](https://github.com/iamlab-cmu/DeltaHands/blob/main/Arduino/delta-hand-servo-serial/delta-hand-servo-serial.ino) includes low-level actuator PID control and communication between the microcontroller and the control pc.

Code for Delta robots' and hands' [kinematics](https://github.com/iamlab-cmu/DeltaHands/blob/main/Teleoperation/Kinematics_Hand.py) and [communication](https://github.com/iamlab-cmu/DeltaHands/blob/main/Teleoperation/DeltaHand.py) between the micro-controller and the control pc.

### Simulation
[A URDF generator](https://github.com/iamlab-cmu/DeltaHands/tree/main/Simulation/urdf_generator) to generate Delta hand models.

[A PyBullet simulation environment](https://github.com/iamlab-cmu/DeltaHands/blob/main/Simulation/grasp.py) for manipulation.

### Teleoperation
[Teleoperation](https://github.com/iamlab-cmu/DeltaHands/blob/main/Teleoperation/leap_teleop.py) based on a leap motion camera.

## Tilde: Teleoperation for Dexterous In-Hand Manipulation Learning with a DeltaHand
Tilde is an imitation learning-based in-hand manipulation system extended from DELTAHANDS. For more information, please see our [paper](https://arxiv.org/abs/2405.18804) or the [website](https://sites.google.com/view/tilde-).

### Control
Arduino code includes the communication between 1) the [DeltaHand](https://github.com/iamlab-cmu/DeltaHands/blob/main/Arduino/delta-hand-servo-rosserial/delta-hand-servo-rosserial.ino) and the control pc, and 2) the [TeleHand](https://github.com/iamlab-cmu/DeltaHands/blob/main/Arduino/tele-hand-rosserial/tele-hand-rosserial.ino) and the control pc.

### Teleoperation
[Teleoperation](https://github.com/iamlab-cmu/DeltaHands/blob/main/Teleoperation/telehand_teleop.py) based on the kinematic twin interface, TeleHand.

### Imitation learning for dexterous in-hand manipulation
Learning framework is built base on [Diffusion Policy](https://diffusion-policy.cs.columbia.edu/). [Data preprocessing](https://github.com/iamlab-cmu/DeltaHands/blob/main/Manipulation/data_processing/rosbag2pkl.py), [training](https://github.com/iamlab-cmu/DeltaHands/blob/main/Manipulation/training.ipynb) and [inference](https://github.com/iamlab-cmu/DeltaHands/blob/main/Manipulation/inference_dagger.py) code are included.


## License
DELTAHANDS and Tilde are licensed under [MIT license](LICENSE).

## Cite DELTAHANDS and Tilde
If you use DELTAHANDS in your research, please cite:
```BibTeX
@article{si2024deltahands,
  title={DELTAHANDS: A Synergistic Dexterous Hand Framework Based on Delta Robots},
  author={Si, Zilin and Zhang, Kevin and Kroemer, Oliver and Temel, F Zeynep},
  journal={IEEE Robotics and Automation Letters},
  year={2024},
  publisher={IEEE}
}
```
If you use Tilde in your research, please cite:
```BibTeX
@article{si2024tilde,
  title={Tilde: Teleoperation for Dexterous In-Hand Manipulation Learning with a DeltaHand},
  author={Si, Zilin and Zhang, Kevin Lee and Temel, Zeynep and Kroemer, Oliver},
  journal={arXiv preprint arXiv:2405.18804},
  year={2024}
}
```
