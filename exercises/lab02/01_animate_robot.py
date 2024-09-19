#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-18
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import numpy as np

from robotics_toolbox.core import SE2
from robotics_toolbox.robots import PlanarManipulator
from robotics_toolbox.render import RendererPlanar

np.random.seed(0)

"""Create robot and a renderer in matplotlib"""
robot = PlanarManipulator(
    link_parameters=[0.3] * 3,
    structure="PRR",
    base_pose=SE2(translation=(0, -0.5)),
)
renderer = RendererPlanar(lim_scale=1.5)

"""PLot the manipulator"""
# joints are either circles (rotation) or squares (prismatic)
renderer.plot_manipulator(robot)

# plot the flange pose
pose_flange = robot.flange_pose()
renderer.plot_se2(pose_flange)

"""Interpolate between the following two configurations"""
q0 = np.random.uniform(low=-np.pi, high=np.pi, size=robot.dof)
q1 = np.random.uniform(low=-np.pi, high=np.pi, size=robot.dof)

for t in np.linspace(0, 1, num=50):
    robot.q = q0 + t * (q1 - q0)  # interpolate

    # update flange pose
    pose_flange.translation = robot.flange_pose().translation
    pose_flange.rotation = robot.flange_pose().rotation

    renderer.redraw_all()

renderer.wait_for_close()
