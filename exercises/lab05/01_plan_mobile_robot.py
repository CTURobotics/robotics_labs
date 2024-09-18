#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-21
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import time

from shapely import MultiPolygon, Point

from robotics_toolbox.core import SE2
from robotics_toolbox.planning.rrt import RRT
from robotics_toolbox.render import RendererPlanar
from robotics_toolbox.robots import MobileRobot
from robotics_toolbox.utils import (
    save_fig,
    create_gif_from_mp4,
    create_mp4_from_folder,
)

robot = MobileRobot()
robot.obstacles = MultiPolygon([Point((0.0, 0.0)).buffer(0.3, cap_style=3)])

render = RendererPlanar(lim_scale=2.0)
render.plot_mobile_robot(robot)

start_state = SE2(translation=[-1, -1])
goal_state = SE2(translation=[0.75, 0.75])

render.plot_se2(start_state)
render.plot_se2(goal_state)

rrt = RRT(robot=robot, delta_q=0.25, p_sample_goal=0.5)
plan = rrt.plan(start_state, goal_state)

if len(plan) == 0:
    print("Solution was not found.")
    exit(1)


for q in plan:
    robot.set_configuration(q)
    render.redraw_all()
    time.sleep(0.5)
    save_fig()

create_gif_from_mp4(create_mp4_from_folder(fps=1))
render.wait_for_close()
