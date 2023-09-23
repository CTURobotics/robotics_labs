#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-5
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D

from robotics_toolbox.core import SE2
from robotics_toolbox.robots.mobile_robot import MobileRobot


class MobileRobotRenderer:
    def __init__(self, ax: plt.Axes, robot: MobileRobot) -> None:
        super().__init__()
        self.robot = robot
        self.ax = ax

        self.img = plt.imread(
            Path(__file__).parent.joinpath("data").joinpath("mobile_robot.png")
        )
        self.im = ax.imshow(self.img, transform=self._transform_from_pose(robot.pose))
        if self.robot.obstacles is not None:
            for p in list(self.robot.obstacles.geoms):
                self.ax.fill(*p.exterior.xy, color="tab:grey")

    def update(self):
        self.im.set_transform(self._transform_from_pose(self.robot.pose))

    def _transform_from_pose(self, pose: SE2):
        h, w = self.img.shape[:2]
        assert h == w, "Mobile robot template needs to be a square image."
        scale = self.robot.size / w
        rs = self.robot.size
        return (
            Affine2D()
            .scale(scale)
            .rotate_around(rs / 2, rs / 2, pose.rotation.angle)
            .translate(*(pose.translation - rs / 2))
            + self.ax.transData
        )
