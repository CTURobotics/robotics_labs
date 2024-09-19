#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-08-21
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

from __future__ import annotations
import matplotlib.pyplot as plt
from numpy.typing import ArrayLike
import numpy as np

from robotics_toolbox.core import SO2, SE2


class SE2Renderer:
    def __init__(
        self, ax: plt.Axes, t: SE2 | SO2, length: float = 0.1, *args, **kwargs
    ) -> None:
        super().__init__()
        self.ax = ax
        self.t = t
        self.length = length

        o = self.t.act(np.array([0, 0]))
        x = self.t.act(np.array([self.length, 0]))
        y = self.t.act(np.array([0, self.length]))
        self.data = [
            self.plot_line_between_points(o, x, "r-", *args, **kwargs)[0],
            self.plot_line_between_points(o, y, "g-", *args, **kwargs)[0],
        ]

    def update(self):
        o = self.t.act(np.array([0, 0]))
        x = self.t.act(np.array([self.length, 0]))
        y = self.t.act(np.array([0, self.length]))
        self.data[0].set_data([o[0], x[0]], [o[1], x[1]])
        self.data[1].set_data([o[0], y[0]], [o[1], y[1]])

    def plot_line_between_points(self, a: ArrayLike, b: ArrayLike, *args, **kwargs):
        """Plot line between two given 2D points a and b. Other arguments passed to
        ax.plot function."""
        return self.ax.plot((a[0], b[0]), (a[1], b[1]), *args, **kwargs)
