#!/usr/bin/env python3

from visual_kinematics.RobotSerial import *
from visual_kinematics.RobotTrajectory import *
import numpy as np
from math import pi


def main():
    np.set_printoptions(precision=3, suppress=True)
    #d,a,alpha,theta
    dh_params = np.array([[0.1623, 0., -0.5 * pi, 0.166 * pi],
                          [0., 0., pi, 0.33 * pi],
                          [0., 0., 0, 0.],
                          [0.2013, 0., -0.5 * pi, 0.25 * pi],
                          [0.1025, 0., 0.5 * pi, 0.41 * pi],
                          [0.015, 0., 0., 0.5 * pi]])

    robot = RobotSerial(dh_params)

    # =====================================
    # trajectory
    # =====================================
#Frame.from_euler_3(np.array([0.25 * pi, 0., 0.75 * pi]), np.array([[0.48127], [0.], [1.13182]])),
    frames = [Frame.from_euler_3(np.array([0.5 * pi, 0.33*pi, pi]), np.array([[0.28127], [0.], [1.13182]])),
              Frame.from_euler_3(np.array([0.25 * pi, 0, 0.75 * pi]), np.array([[0.48127], [0.], [1.13182]])),
              Frame.from_euler_3(np.array([0.5 * pi, 0, pi]), np.array([[0.48127], [0.], [0.63182]]))]
    trajectory = RobotTrajectory(robot, frames)
    trajectory.show(motion="p2p")


if __name__ == "__main__":
    main()