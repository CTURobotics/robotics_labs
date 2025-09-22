#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import numpy as np
from robotics_toolbox.core import SE2, SO2
from robotics_toolbox.render import RendererPlanar
from robotics_toolbox.robots import PlanarManipulator
from robotics_toolbox.utils import (
    save_fig,
    create_gif_from_mp4,
    create_mp4_from_folder,
)

robot = PlanarManipulator(
    link_parameters=[0.3] * 5,
    base_pose=SE2([-0.75, 0.0]),
    structure="RRRRR",
)
render = RendererPlanar(lim_scale=2.0)
render.plot_manipulator(robot)

desired_pose = SE2(translation=[-1.5, 1.0], rotation=SO2(angle=np.deg2rad(45 + 90)))
render.plot_se2(desired_pose)


for _ in range(50):
    save_fig()
    converged = robot.ik_numerical(flange_pose_desired=desired_pose, max_iterations=1)
    render.plot_manipulator(robot)
    if converged:
        save_fig()
        print("Solution found.")
        break
create_gif_from_mp4(create_mp4_from_folder(fps=1))
render.wait_for_close()
