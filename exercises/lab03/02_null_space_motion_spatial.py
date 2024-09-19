#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import numpy as np
from robotics_toolbox.render import RendererSpatial
from robotics_toolbox.robots import SpatialManipulator
from robotics_toolbox.utils import nullspace

robot = SpatialManipulator(robot_name="panda")
robot.q[0] = -np.pi / 2
robot.q[1] = -np.pi / 4
robot.q[3] = -np.pi / 2
robot.q[5] = np.pi / 2
render = RendererSpatial()
render.camera_zoom = 2.0

render.plot_manipulator(robot)

with render.animation():
    for i in range(100):
        jac = robot.jacobian()
        ns = nullspace(jac)
        k = -1 if i > 50 else 1
        robot.q += k * 0.025 * ns[:, 0]
        render.plot_manipulator(robot)
render.wait_at_the_end()
