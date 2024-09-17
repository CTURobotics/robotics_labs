#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-12-2
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

import numpy as np
from robotics_toolbox.render import RendererPlanar
from robotics_toolbox.robots.planar_manipulator_dynamics import (
    PlanarManipulatorDynamics,
)
from robotics_toolbox.utils import (
    save_fig,
    create_gif_from_mp4,
    create_mp4_from_folder,
)

r = PlanarManipulatorDynamics(
    link_parameters=[0, 1.0],
    structure="PR",
    masses=[0.1, 0.5],
)
render = RendererPlanar(lim_scale=2.0)
render.plot_manipulator(r)

q = r.q
dq = np.zeros(r.dof)
tau = np.zeros(r.dof)

dt = 0.0005

render.plot_line_between_points(
    r.flange_pose().translation - [10, 10],
    r.flange_pose().translation + [10, 10],
    color="red",
    linewidth=2,
)
for i in range(20000):
    ddq = r.constrained_forward_dynamics(q, dq, tau, damping=0)
    dq += ddq * dt
    q += dq * dt
    r.q = q
    if i % 100 == 0:
        render.plot_manipulator(r)
        save_fig()

create_gif_from_mp4(create_mp4_from_folder(fps=20))

render.wait_for_close()
