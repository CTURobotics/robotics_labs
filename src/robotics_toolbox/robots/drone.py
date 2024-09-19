#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-7
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from __future__ import annotations
import numpy as np

from robotics_toolbox.core import SE3, SE2, SO3
from robotics_toolbox.robots.robot_base import RobotBase


class Drone(RobotBase):
    def __init__(self) -> None:
        super().__init__()
        self.pose = SE3()
        self.min_translation = -5.0 * np.ones(3)
        self.max_translation = 5.0 * np.ones(3)

    def sample_configuration(self) -> np.ndarray | SE2 | SE3:
        return SE3(
            translation=np.random.uniform(self.min_translation, self.max_translation),
            rotation=SO3.exp(np.random.uniform(0, np.pi, size=3)),
        )

    def set_configuration(self, configuration: np.ndarray | SE2 | SE3):
        self.pose = configuration
        return self

    def in_collision(self) -> bool:
        return False

    def configuration(self) -> np.ndarray | SE2 | SE3:
        return self.pose
