#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-18
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

import numpy as np

from robotics_toolbox.robots import SpatialManipulator
from robotics_toolbox.render import RendererSpatial

np.random.seed(0)

renderer = RendererSpatial()
robot = SpatialManipulator(robot_name="panda")

renderer.plot_manipulator(robot)
frame = robot.flange_pose()
renderer.plot_se3(frame)

"""Interpolate between the following two configurations"""
q0 = np.random.uniform(low=-np.pi, high=np.pi, size=robot.dof)
q1 = np.random.uniform(low=-np.pi, high=np.pi, size=robot.dof)

"""Let's not animate gripper."""
q0[-2:] = 0
q1[-2:] = 0

with renderer.animation(fps=10):
    for t in np.linspace(0, 1, num=50):
        robot.q = q0 + t * (q1 - q0)  # interpolate
        renderer.plot_manipulator(robot, render=False)
        frame.set_from(robot.flange_pose())
        renderer.plot_se3(frame)

renderer.wait_at_the_end()
