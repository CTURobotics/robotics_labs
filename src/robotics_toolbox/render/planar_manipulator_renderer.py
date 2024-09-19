#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-08-21
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from numpy.typing import ArrayLike
import matplotlib.pyplot as plt

from robotics_toolbox.robots import PlanarManipulator


class PlanarManipulatorRenderer:
    def __init__(self, ax: plt.Axes, robot: PlanarManipulator, **kwargs) -> None:
        super().__init__()
        self.robot = robot
        self.ax = ax
        self.plt_objects = {}
        self.plot_init(**kwargs)

    def update(self):
        frames = self.robot.fk_all_links()
        gripper = self.robot._gripper_lines(frames[-1])
        for p, t in zip(self.plt_objects["joints"], frames[:-1]):
            p.set_data(*t.translation)
        for p, t, tn in zip(self.plt_objects["links"], frames[:-1], frames[1:]):
            p.set_data(
                [t.translation[0], tn.translation[0]],
                [t.translation[1], tn.translation[1]],
            )
        for p, (a, b) in zip(self.plt_objects["gripper"], gripper):
            p.set_data([a[0], b[0]], [a[1], b[1]])

    def plot_init(self, color="k", lw=3, ms=7):
        """Plot the robot. It returns the plt_objects, that you can use to update the
        robot pose in animation."""
        frames = self.robot.fk_all_links()
        gripper = self.robot._gripper_lines(frames[-1])
        self.plt_objects = {
            "joints": [
                self.ax.plot(
                    *t.translation,
                    "o" if jtype.lower() == "r" else "s",
                    ms=ms,
                    color=color,
                )[0]
                for t, jtype in zip(frames[:-1], self.robot.structure)
            ],
            "links": [
                self.plot_line_between_points(
                    t.translation, tn.translation, color=color, lw=lw
                )[0]
                for t, tn in zip(frames[:-1], frames[1:])
            ],
            "gripper": [
                self.plot_line_between_points(a, b, color=color, lw=lw)[0]
                for a, b in gripper
            ],
        }
        if self.robot.obstacles is not None:
            for p in list(self.robot.obstacles.geoms):
                self.ax.fill(*p.exterior.xy, color="tab:grey")

    def plot_line_between_points(self, a: ArrayLike, b: ArrayLike, *args, **kwargs):
        """Plot line between two given 2D points a and b. Other arguments passed to
        ax.plot function."""
        return self.ax.plot((a[0], b[0]), (a[1], b[1]), *args, **kwargs)
