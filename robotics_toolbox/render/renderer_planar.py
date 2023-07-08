#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-4
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import ArrayLike

from robotics_toolbox.core import SO2, SE2
from robotics_toolbox.render.mobile_robot_renderer import MobileRobotRenderer
from robotics_toolbox.robots.mobile_robot import MobileRobot


class RendererPlanar:
    def __init__(
        self, xlim: tuple[float, float] = (-1, 1), ylim: tuple[float, float] = (-1, 1)
    ) -> None:
        super().__init__()
        self.ylim = ylim
        self.xlim = xlim
        self.fig, self.ax = plt.subplots(
            1, 1, squeeze=True
        )  # type: plt.Figure, plt.Axes
        self.ax.axis("equal")
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        plt.ion()
        plt.show()
        self._redraw()

        self.mobile_robots: dict[MobileRobot, MobileRobotRenderer] = {}

    def _redraw(self):
        self.ax.set_xlim(*self.xlim)
        self.ax.set_ylim(*self.ylim)
        self.fig.canvas.draw()
        plt.pause(0.05)

    def plot_line_between_points(self, a: ArrayLike, b: ArrayLike, *args, **kwargs):
        """Plot line between two given 2D points a and b. Other arguments passed to
        ax.plot function."""
        self.ax.plot((a[0], b[0]), (a[1], b[1]), *args, **kwargs)
        self._redraw()

    @staticmethod
    def wait_for_enter(msg: str | None = None):
        if msg is None:
            msg = "Press enter to continue."
        input(msg)

    def plot_so2(self, t: SO2, length=0.1, *args, **kwargs):
        """Plot SO2 frame in the origin."""
        self.plot_se2(SE2(rotation=t), length=length, *args, **kwargs)

    def plot_se2(self, t: SE2, length=0.1, *args, **kwargs):
        """Plot SE2 frame."""
        o = t.act(np.array([0, 0]))
        x = t.act(np.array([length, 0]))
        y = t.act(np.array([0, length]))
        self.plot_line_between_points(o, x, "r-", *args, **kwargs)
        self.plot_line_between_points(o, y, "g-", *args, **kwargs)
        self._redraw()

    def plot_mobile_robot(self, robot: MobileRobot):
        """Plot mobile robot into the figure. If this function is called multiple times
        for the same robot, this function redraw the robot to the new pose instead
        of drawing a new one."""
        if robot in self.mobile_robots:
            self.mobile_robots[robot].update(robot)
        else:
            self.mobile_robots[robot] = MobileRobotRenderer(self.ax, robot)
        self._redraw()

    @staticmethod
    def wait_for_close():
        plt.ioff()
        plt.show()
