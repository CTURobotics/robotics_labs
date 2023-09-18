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

"""Interpolate between the following two configurations"""
q0 = np.zeros(robot.dof)
q1 = np.ones(robot.dof)

with renderer.animation(fps=10):
    for t in np.linspace(0, 1, num=50):
        robot.q = q0 + t * (q1 - q0)  # interpolate
        renderer.plot_manipulator(robot)

renderer.wait_at_the_end()
