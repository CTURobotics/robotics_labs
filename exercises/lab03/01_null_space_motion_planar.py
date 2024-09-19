#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from robotics_toolbox.core import SE2
from robotics_toolbox.render import RendererPlanar
from robotics_toolbox.robots import PlanarManipulator
from robotics_toolbox.utils import (
    nullspace,
    save_fig,
    create_gif_from_mp4,
    create_mp4_from_folder,
)

robot = PlanarManipulator(
    link_parameters=[0.4] * 4,
    base_pose=SE2([-0.75, 0.0]),
    structure="RRRR",
)
render = RendererPlanar(lim_scale=2.0)

render.plot_manipulator(robot)

for i in range(100):
    jac = robot.jacobian()
    ns = nullspace(jac)
    k = -1 if i > 50 else 1
    robot.q += k * 0.025 * ns[:, 0]
    render.redraw_all()
    save_fig()

create_gif_from_mp4(create_mp4_from_folder())
