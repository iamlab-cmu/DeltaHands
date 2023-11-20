# DELTAHANDS: A Synergistic Dexterous Hand Framework Based on Delta Robots
DELTAHANDS is a synergistic dexterous hand framework that utilizes soft Delta robots. For more information on DELTAHANDS, please see our [paper](https://arxiv.org/abs/2310.05266) or the [website](https://sites.google.com/view/deltahands/).

## Control
[Arduino code](https://github.com/iamlab-cmu/DeltaHands/tree/main/Arduino) includes low-level actuator PID control and communication between the microcontroller and the control pc.

Python code includes Delta robots' and hands' [kinematics](https://github.com/iamlab-cmu/DeltaHands/blob/main/Teleoperation/Kinematics_Hand.py) and [communication](https://github.com/iamlab-cmu/DeltaHands/blob/main/Teleoperation/DeltaHand.py) between the micro-controller and the control pc.

## Simulation
[A URDF generator](https://github.com/iamlab-cmu/DeltaHands/tree/main/Simulation/urdf_generator) to generate Delta hand models.

[A PyBullet simulation environment](https://github.com/iamlab-cmu/DeltaHands/blob/main/Simulation/grasp.py) for manipulation.

## Teleoperation
[Teleoperation](https://github.com/iamlab-cmu/DeltaHands/blob/main/Teleoperation/realtime_teleop_viz.py) based on leap motion camera.

## License
DELTAHANDS is licensed under [MIT license](LICENSE).

## Cite DELTAHANDS
If you use DELTAHANDS in your research, please cite:
```BibTeX
@article{si2023deltahands,
  title={DELTAHANDS: A Synergistic Dexterous Hand Framework Based on Delta Robots},
  author={Si, Zilin and Zhang, Kevin and Kroemer, Oliver and Temel, F Zeynep},
  journal={arXiv preprint arXiv:2310.05266},
  year={2023}
}
```
