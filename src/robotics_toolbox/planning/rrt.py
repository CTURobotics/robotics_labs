#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-21
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from __future__ import annotations
import numpy as np
from numpy.typing import ArrayLike
from copy import deepcopy

from robotics_toolbox.core import SE3, SE2
from robotics_toolbox.robots.robot_base import RobotBase


class RRT:
    def __init__(self, robot: RobotBase, delta_q=0.2, p_sample_goal=0.5) -> None:
        """RRT planner for a given robot.
        Args:
            robot: robot used to sample configuration and check collisions
            delta_q: maximum distance between two configurations
            p_sample_goal: probability of sampling goal as q_rand
        """
        self.p_sample_goal = p_sample_goal
        self.robot = robot
        self.delta_q = delta_q

    def plan(
        self,
        q_start: ArrayLike | SE2 | SE3,
        q_goal: ArrayLike | SE2 | SE3,
        max_iterations: int = 10000,
    ) -> list[ArrayLike | SE2 | SE3]:
        """RRT algorithm for motion planning."""
        assert not self.robot.set_configuration(q_start).in_collision()
        assert not self.robot.set_configuration(q_goal).in_collision()
        # todo: hw06opt implement RRT
        return []

    def random_shortcut(
        self, path: list[np.ndarray | SE2 | SE3], max_iterations=100
    ) -> list[np.ndarray | SE2 | SE3]:
        """Random shortcut algorithm that pick two points on the path randomly and tries
        to interpolate between them. If collision free interpolation exists,
        the path between selected points is replaced by the interpolation."""
        # todo: hw06opt implement random shortcut algorithm
        out = deepcopy(path)
        return out
