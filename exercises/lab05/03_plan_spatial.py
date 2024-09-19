#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-21
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import numpy as np
from robotics_toolbox.planning.rrt import RRT
from robotics_toolbox.render import RendererSpatial
from robotics_toolbox.robots import SpatialManipulator

# from robotics_toolbox.utils import (
#     save_fig,
#     create_gif_from_mp4,
#     create_mp4_from_folder,
# )


robot = SpatialManipulator(robot_name="panda")
robot.q[0] = -np.pi / 2
robot.q[1] = -np.pi / 4
robot.q[3] = -np.pi / 2
robot.q[5] = np.pi / 2

# Robot in collision:
# robot.q[3] = -np.pi
# robot.q[5] = np.pi / 2 - np.pi / 4
# print(robot.in_collision())

render = RendererSpatial()
render.camera_zoom = 2.0

render.plot_manipulator(robot)
render.wait_at_the_end()

start_state = robot.q.copy()
goal_state = robot.q.copy()
goal_state[0] = np.pi / 2
goal_state[1] = np.pi / 2


rrt = RRT(robot=robot, delta_q=0.2, p_sample_goal=0.5)
plan = rrt.plan(start_state, goal_state)

if len(plan) == 0:
    print("Solution was not found.")
    exit(1)

with render.animation(fps=1):
    for q in plan:
        robot.set_configuration(q)
        render.plot_manipulator(robot)
    # save_fig(renderer=render)
# create_gif_from_mp4(create_mp4_from_folder(fps=5))
