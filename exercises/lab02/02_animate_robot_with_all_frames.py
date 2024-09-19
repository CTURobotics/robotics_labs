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
from robotics_toolbox.utils import save_fig, create_mp4_from_folder, create_gif_from_mp4

np.random.seed(0)

"""Create robot and a renderer in matplotlib"""
robot = PlanarManipulator(
    link_parameters=[0.4] * 4,
    structure="PRRP",
    base_pose=SE2(translation=(0, -0.5)),
)
renderer = RendererPlanar(lim_scale=2.0)

"""PLot the manipulator"""
renderer.plot_manipulator(robot)

frames = robot.fk_all_links()
for f in frames:
    renderer.plot_se2(f, length=0.3)

"""Interpolate between the following two configurations"""
q0 = np.zeros(robot.dof)

for i in range(robot.dof):
    for _ in range(10):
        robot.q[i] += 0.1 if robot.structure[i] == "R" else 0.05

        # update flange pose
        for f_to, f_from in zip(frames, robot.fk_all_links()):
            f_to.translation = f_from.translation
            f_to.rotation = f_from.rotation

        renderer.redraw_all()

        save_fig("/tmp/animation")
create_gif_from_mp4(create_mp4_from_folder("/tmp/animation"))


renderer.wait_for_close()
