#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-7
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

from robotics_toolbox.core import SE3, SO3
from robotics_toolbox.render.renderer_spatial import RendererSpatial
from robotics_toolbox.robots.drone import Drone

renderer = RendererSpatial()
robot = Drone()
renderer.plot_drone(robot)  # we need to plot drone before animation

with renderer.animation(fps=10):
    # move up (i.e. positive z-axis)
    for _ in range(10):
        robot.pose = robot.pose * SE3(translation=[0, 0, 0.1])
        renderer.plot_drone(robot)

    # rotate around z-axis
    for _ in range(10):
        robot.pose = robot.pose * SE3(rotation=SO3.exp([0, 0, -0.1]))
        renderer.plot_drone(robot)

    # tilt slightly
    for _ in range(10):
        robot.pose = robot.pose * SE3(rotation=SO3.exp([0.0, 0.05, 0.0]))
        renderer.plot_drone(robot)

    # move forward
    for _ in range(10):
        robot.pose = robot.pose * SE3(translation=[0.1, 0.0, 0])
        renderer.plot_drone(robot)
