#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-21
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import numpy as np
from copy import deepcopy
from shapely import MultiPolygon, Point

from robotics_toolbox.core import SE2
from robotics_toolbox.planning.rrt import RRT
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
robot.obstacles = MultiPolygon([Point((0.5, 0.5)).buffer(0.3, cap_style=3)])
render.plot_manipulator(robot)

start_state = -np.pi / 4 * np.ones(robot.dof)
goal_state = np.pi / 4 * np.ones(robot.dof)

start_robot = deepcopy(robot)
start_robot.set_configuration(start_state)
render.plot_manipulator(start_robot, color="tab:green")
goal_robot = deepcopy(robot)
goal_robot.set_configuration(goal_state)
render.plot_manipulator(goal_robot, color="tab:red")

rrt = RRT(robot=robot, delta_q=0.2, p_sample_goal=0.5)
path = rrt.plan(start_state, goal_state)
if len(path) == 0:
    print("Solution was not found.")
    exit(1)


# Plot path with the end-effector positions
for q_last, q in zip([None] + path, path):
    if q_last is not None:
        a = robot.set_configuration(q_last).flange_pose().translation
        b = robot.set_configuration(q).flange_pose().translation
        render.plot_line_between_points(a, b, color="tab:orange")
    robot.set_configuration(q)
    render.redraw_all()
    save_fig()
create_gif_from_mp4(create_mp4_from_folder(fps=5))

path_simplified = rrt.random_shortcut(path)

# Plot simplified with the end-effector positions
for q_last, q in zip([None] + path_simplified, path_simplified):
    if q_last is not None:
        a = robot.set_configuration(q_last).flange_pose().translation
        b = robot.set_configuration(q).flange_pose().translation
        render.plot_line_between_points(a, b, color="tab:blue")
    robot.set_configuration(q)
    render.redraw_all()
    save_fig(output_folder="/tmp/animation2")
create_gif_from_mp4(create_mp4_from_folder(fps=5, folder="/tmp/animation2"))

render.wait_for_close()
