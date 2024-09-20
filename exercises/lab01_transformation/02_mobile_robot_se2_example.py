#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-5
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
# The pose of the mobile robot that moves on the plane is described by SE2
# transformation. This example shows how composition of the transformation can be
# useful to integrate robot motion or to transfer his local observation to the map
# reference frame.
from time import sleep

from robotics_toolbox.core import SE2, SO2
from robotics_toolbox.render import RendererPlanar
from robotics_toolbox.robots.mobile_robot import MobileRobot

renderer = RendererPlanar()

robot = MobileRobot()
animation_speed = 0.01

# the forward motion of the robot can be interpreted as translation applied 'from' the
# right side, i.e. robot_pose = old_robot_pose * translation_x(value)
# Let's animate the translation of the robot:
for _ in range(10):
    robot.pose = robot.pose * SE2(translation=[0.025, 0.0])
    renderer.plot_mobile_robot(robot)
    sleep(animation_speed)

# note, that in this specific case (i.e. robot's angle == 0) multiplication from left
# and right gives the same result, i.e:
for _ in range(10):
    robot.pose = SE2(translation=[0.025, 0.0]) * robot.pose
    renderer.plot_mobile_robot(robot)
    sleep(animation_speed)

# The rotation of the robot around it's axis can be achieved in the same way
for _ in range(10):
    robot.pose = robot.pose * SE2(rotation=SO2(0.1))
    # robot.pose = SE2(rotation=SO2(0.1)) * robot.pose # this is incorrect, try it.
    renderer.plot_mobile_robot(robot)
    sleep(animation_speed)

# here changing the order of multiplication would have different result!

# The local translation can be then applied again
for _ in range(10):
    robot.pose = robot.pose * SE2(translation=[0.025, 0.0])
    # robot.pose = SE2(translation=[0.025, 0.0]) * robot.pose # this is incorrect
    renderer.plot_mobile_robot(robot)
    sleep(animation_speed)
# here the order of multiplication matters as well.


# --- Transformation of observations from the camera into the world frame. ---
# Robot 'see' objects in its own reference frame, for example, a point [0.9, 0.75] in
# our map frame will robot see as:
renderer.plot_se2(SE2())
renderer.plot_se2(robot.pose)
p_map = [0.9, 0.75]
renderer.ax.plot(*p_map, "o", ms=10, color="tab:green")
p_robot = robot.pose.inverse().act(p_map)
# If we know detection in robot frame, we can easily transfer to our map frame:
# p_map_computed = robot.pose.act(p_robot)

print(f"Point w.r.t. map frame: {p_map}")
print(f"Point w.r.t. robot frame: {p_robot}")

renderer.wait_for_close()
