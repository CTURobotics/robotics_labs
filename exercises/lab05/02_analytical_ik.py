#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from copy import deepcopy

import matplotlib

from robotics_toolbox.core import SE2
from robotics_toolbox.render import RendererPlanar
from robotics_toolbox.render.planar_manipulator_renderer import (
    PlanarManipulatorRenderer,
)
from robotics_toolbox.robots import PlanarManipulator
from robotics_toolbox.utils import save_fig

robot = PlanarManipulator(
    link_parameters=[0.5] * 3,
    base_pose=SE2([-0.5, -0.75]),
    structure="RRR",
    # structure="PRR", # for optional HW
)
render = RendererPlanar(lim_scale=2.0)

desired_pose = robot.flange_pose()
# desired_pose = SE2(translation=[-0.75, 0.25], rotation=SO2(angle=np.deg2rad(45 + 90)))

render.plot_se2(desired_pose)

solutions = robot.ik_analytical(desired_pose)
if len(solutions) == 0:
    print("No solution found.")

# Display all solutions with different colors
clrs = matplotlib.colormaps["tab10"].colors
for i, sol in enumerate(solutions):
    r = deepcopy(robot)
    r.q = sol
    render.manipulators[r] = PlanarManipulatorRenderer(render.ax, r, color=clrs[i])
    render.plot_manipulator(r)
    save_fig()

render.wait_for_close()
