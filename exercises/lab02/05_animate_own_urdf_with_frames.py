#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-18
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from pathlib import Path

import numpy as np

from robotics_toolbox.robots import SpatialManipulator
from robotics_toolbox.render import RendererSpatial

np.random.seed(0)

renderer = RendererSpatial()
robot = SpatialManipulator(urdf_path=Path(__file__).parent.joinpath("robot.urdf"))

renderer.plot_manipulator(robot)

"""Plot frames of all links of the robot."""
frames = list(robot.link_poses().values())
for f in frames:
    renderer.plot_se3(f, render=False)

"""Interpolate between the following two configurations"""

q0 = robot.sample_configuration()
q1 = robot.sample_configuration()

with renderer.animation(fps=10):
    for t in np.linspace(0, 1, num=50):
        robot.q = q0 + t * (q1 - q0)  # interpolate

        for f, pose in zip(frames, robot.link_poses().values()):
            f.set_from(pose)
            renderer.plot_se3(f, render=False)
        renderer.plot_manipulator(robot)

renderer.wait_at_the_end()
