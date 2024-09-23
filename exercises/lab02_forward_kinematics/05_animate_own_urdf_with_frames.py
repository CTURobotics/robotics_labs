#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-18
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from pathlib import Path

import numpy as np
import pinocchio as pin

from robotics_toolbox.core import SE3, SO3
from robotics_toolbox.robots import SpatialManipulator
from robotics_toolbox.render import RendererSpatial

np.random.seed(0)

renderer = RendererSpatial()
robot = SpatialManipulator(urdf_path=Path(__file__).parent.joinpath("robot.urdf"))

renderer.plot_manipulator(robot)

frames_hom = [robot._data.oMf[fid].homogeneous for fid in range(robot._model.nframes)]
frames = [SE3(f[:3, 3], SO3(f[:3, :3])) for f in frames_hom]
for f in frames:
    renderer.plot_se3(f, render=False)

"""Interpolate between the following two configurations"""

q0 = robot.sample_configuration()
q1 = robot.sample_configuration()

with renderer.animation(fps=10):
    for t in np.linspace(0, 1, num=50):
        robot.q = q0 + t * (q1 - q0)  # interpolate
        pin.forwardKinematics(robot._model, robot._data, robot.q)
        pin.updateFramePlacements(robot._model, robot._data)

        for i in range(robot._model.nframes):
            f = robot._data.oMf[i].homogeneous
            frames[i].set_from(SE3(f[:3, 3], SO3(f[:3, :3])))
            renderer.plot_se3(frames[i], render=False)
        renderer.plot_manipulator(robot)

renderer.wait_at_the_end()
