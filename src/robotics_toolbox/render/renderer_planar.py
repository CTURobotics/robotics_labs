#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-4
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import ArrayLike

from robotics_toolbox.core import SO2, SE2
from robotics_toolbox.render.mobile_robot_renderer import MobileRobotRenderer
from robotics_toolbox.render.planar_manipulator_renderer import (
    PlanarManipulatorRenderer,
)
from robotics_toolbox.render.se2_renderer import SE2Renderer
from robotics_toolbox.robots import PlanarManipulator, MobileRobot


class RendererPlanar:
    def __init__(
        self,
        xlim: tuple[float, float] = (-1, 1),
        ylim: tuple[float, float] = (-1, 1),
        lim_scale: float = 1.0,
    ) -> None:
        super().__init__()
        self.ylim = np.asarray(ylim) * lim_scale
        self.xlim = np.asarray(xlim) * lim_scale
        self.fig, self.ax = plt.subplots(
            1, 1, squeeze=True
        )  # type: plt.Figure, plt.Axes
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        plt.ion()
        plt.show()
        self._redraw()

        self.mobile_robots: dict[MobileRobot, MobileRobotRenderer] = {}
        self.manipulators: dict[PlanarManipulator, PlanarManipulatorRenderer] = {}
        self.se2s: dict[SE2, SE2Renderer] = {}
        self.so2s: dict[SO2, SE2Renderer] = {}

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
        if t in self.so2s:
            self.so2s[t].update()
        else:
            self.so2s[t] = SE2Renderer(self.ax, t, length, *args, **kwargs)
        self._redraw()

    def plot_se2(self, t: SE2, length=0.1, *args, **kwargs):
        """Plot SE2 frame."""
        if t in self.se2s:
            self.se2s[t].update()
        else:
            self.se2s[t] = SE2Renderer(self.ax, t, length, *args, **kwargs)
        self._redraw()

    def plot_mobile_robot(self, robot: MobileRobot):
        """Plot mobile robot into the figure. If this function is called multiple times
        for the same robot, this function redraw the robot to the new pose instead
        of drawing a new one."""
        if robot in self.mobile_robots:
            self.mobile_robots[robot].update()
        else:
            self.mobile_robots[robot] = MobileRobotRenderer(self.ax, robot)
        self._redraw()

    def plot_manipulator(self, robot: PlanarManipulator, **kwargs):
        if robot in self.manipulators:
            self.manipulators[robot].update()
        else:
            self.manipulators[robot] = PlanarManipulatorRenderer(
                self.ax, robot, **kwargs
            )
        self._redraw()

    def redraw_all(self):
        """Redraw all the manipulators that has been plotted before."""
        for o in self.so2s.values():
            o.update()
        for o in self.se2s.values():
            o.update()
        for o in self.mobile_robots.values():
            o.update()
        for o in self.manipulators.values():
            o.update()
        self._redraw()

    @staticmethod
    def wait_for_close():
        plt.ioff()
        plt.show()
