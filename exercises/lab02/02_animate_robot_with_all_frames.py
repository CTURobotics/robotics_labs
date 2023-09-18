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
    link_lengths=[0.4] * 3,
    fixed_rotations=[np.deg2rad(30)] * 3,
    structure="RRR",
    base_pose=SE2(translation=(0, -0.5)),
)
renderer = RendererPlanar(lim_scale=2.0)

"""PLot the manipulator"""
renderer.plot_manipulator(robot)

frames = robot.fk_all_links()
for f in frames:
    renderer.plot_se2(f, length=0.2)

"""Interpolate between the following two configurations"""
q0 = np.zeros(robot.dof)

for i in range(robot.dof):
    for _ in range(50):
        robot.q[i] += 0.1

        # update flange pose
        for f_to, f_from in zip(frames, robot.fk_all_links()):
            f_to.translation = f_from.translation
            f_to.rotation = f_from.rotation

        renderer.redraw_all()

renderer.wait_for_close()
