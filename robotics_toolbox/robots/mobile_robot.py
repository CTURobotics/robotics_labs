#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-5
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from __future__ import annotations
import numpy as np
from shapely import MultiPolygon, Point, affinity

from robotics_toolbox.core import SE2, SE3, SO2
from robotics_toolbox.robots.robot_base import RobotBase


class MobileRobot(RobotBase):
    def __init__(self, size: float = 0.3) -> None:
        super().__init__()
        self.pose = SE2()
        self.size = size
        self.min_position = np.array([-1, -1])
        self.max_position = np.array([1, 1])

        self.obstacles: MultiPolygon | None = None

    def sample_configuration(self) -> np.ndarray | SE2 | SE3:
        return SE2(
            translation=np.random.uniform(self.min_position, self.max_position),
            rotation=SO2(angle=np.random.uniform(-np.pi, np.pi)),
        )

    def set_configuration(self, configuration: np.ndarray | SE2 | SE3):
        self.pose = configuration
        return self

    def in_collision(self) -> bool:
        geometry = Point(self.pose.translation).buffer(self.size, cap_style=3)
        geometry = affinity.rotate(
            geometry, np.rad2deg(self.pose.rotation.angle), "center"
        )
        return geometry.intersects(self.obstacles)

    def configuration(self) -> np.ndarray | SE2 | SE3:
        return self.pose
