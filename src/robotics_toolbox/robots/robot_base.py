#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-21
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from __future__ import annotations
from abc import abstractmethod
import numpy as np

from robotics_toolbox.core import SE2, SE3


class RobotBase:
    @abstractmethod
    def sample_configuration(self) -> np.ndarray | SE2 | SE3:
        """Sample robot configuration inside the configuration space."""
        pass

    @abstractmethod
    def set_configuration(self, configuration: np.ndarray | SE2 | SE3):
        """Set internal configuration to @param configuration. Returns self."""
        pass

    @abstractmethod
    def configuration(self) -> np.ndarray | SE2 | SE3:
        """Get the configuration of the robot, can be array, SE2, or SE3."""
        pass

    @abstractmethod
    def in_collision(self) -> bool:
        """Check if robot is in collision."""
        pass
