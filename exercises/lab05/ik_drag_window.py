#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2026-11-09
#     Author: Martin Cifka <martin.cifka@cvut.cz>
#
# A matplotlib window with a draggable target frame for the interactive IK scripts
# 03_numerical_ik_interactive.py and 04_analytical_ik_interactive.py. Drag the black dot
# to move the target, drag the ring in front of it to rotate it; hold Shift to snap.
# The script supplies the `on_update` callback that solves the IK and draws the robot(s).
from __future__ import annotations

from copy import deepcopy
from typing import Callable

import matplotlib.pyplot as plt
import numpy as np

from robotics_toolbox.core import SE2
from robotics_toolbox.render.planar_manipulator_renderer import (
    PlanarManipulatorRenderer,
)
from robotics_toolbox.render.se2_renderer import SE2Renderer
from robotics_toolbox.robots import PlanarManipulator


def draw_robot_config(
    ax, robot: PlanarManipulator, q, color="k", lw=3.0, alpha=1.0, label=None
):
    """Draw a copy of the robot in configuration q with the toolbox renderer."""
    r = deepcopy(robot)
    r.q = np.asarray(q, float)
    rend = PlanarManipulatorRenderer(ax, r, color=color, lw=lw)
    for objs in rend.plt_objects.values():
        for o in objs:
            o.set_alpha(alpha)
    if label is not None:
        rend.plt_objects["links"][0].set_label(label)
    return rend


class DragTargetWindow:
    """Figure with a draggable target frame (same look as RendererPlanar).

    on_update(app) is called after every change of the target; it should draw into
    app.ax and return a status string shown below the plot.
    on_key(app, key) is called for key presses.
    The base frame of the robot (base_pose) is always drawn, so the base stays visible
    even when there is no solution to draw.
    """

    KNOB_DIST = 0.45  # distance of the rotation ring from the target origin [m]

    def __init__(
        self,
        target: SE2,
        base_pose: SE2 | None = None,
        xlim=(-1, 1),
        ylim=(-1, 1),
        lim_scale: float = 1.0,
        title: str = "",
        help_text: str = "",
    ):
        self.target = target
        self.base_pose = base_pose
        self.on_update: Callable[[DragTargetWindow], str] | None = None
        self.on_key: Callable[[DragTargetWindow, str], None] | None = None

        self.fig, self.ax = plt.subplots(figsize=(8, 9.2))
        self.fig.subplots_adjust(bottom=0.22, top=0.95)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlim(*(np.asarray(xlim) * lim_scale))
        self.ax.set_ylim(*(np.asarray(ylim) * lim_scale))
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.set_title(title)
        self._tol = 0.03 * np.ptp(self.ax.get_xlim())  # grab radius [m]
        self._status = self.fig.text(
            0.08, 0.125, "", family="monospace", fontsize=9.5, va="top"
        )
        self.fig.text(
            0.08,
            0.012,
            "Drag the black dot to move the target, drag the ring to rotate it "
            "(hold Shift to snap to 0.05 m / 15 deg).\n" + help_text,
            fontsize=9,
            color="0.35",
            va="bottom",
        )
        self._drag = None
        self.fig.canvas.mpl_connect("button_press_event", self._on_press)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_motion)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

    # ----------------------------------------------------------------- drawing
    def clear(self):
        """Remove everything drawn into the axes."""
        for art in list(self.ax.lines) + list(self.ax.patches) + list(self.ax.texts):
            art.remove()
        legend = self.ax.get_legend()
        if legend is not None:
            legend.remove()

    def knob(self) -> np.ndarray:
        a = self.target.rotation.angle
        return self.target.translation + self.KNOB_DIST * np.array(
            [np.cos(a), np.sin(a)]
        )

    def draw_target(self):
        SE2Renderer(self.ax, self.target, length=0.25, lw=2.5)
        g, knob = self.target.translation, self.knob()
        self.ax.plot([g[0], knob[0]], [g[1], knob[1]], ":", color="0.6", lw=1)
        self.ax.plot(*knob, "o", mfc="none", mec="0.3", mew=2, ms=11)
        self.ax.plot(*g, "o", color="k", ms=7, zorder=6)

    def draw_base(self):
        if self.base_pose is None:
            return
        SE2Renderer(self.ax, self.base_pose, length=0.2, lw=2)
        o = self.base_pose.translation
        self.ax.plot(*o, "s", color="0.3", ms=9, zorder=5)

    def update(self):
        self.clear()
        self.draw_base()
        self.draw_target()
        status = self.on_update(self) if self.on_update is not None else ""
        self._status.set_text(status)
        self.fig.canvas.draw_idle()

    def set_target(self, target: SE2):
        self.target = target
        self.update()

    def show(self):
        self.update()
        plt.show()

    # ------------------------------------------------------------------ events
    def _hit(self, x, y):
        if np.hypot(*(self.knob() - [x, y])) < self._tol:
            return "knob"
        if np.hypot(*(self.target.translation - [x, y])) < self._tol:
            return "target"
        return None

    def _on_press(self, event):
        if event.inaxes != self.ax or event.button != 1:
            return
        self._drag = self._hit(event.xdata, event.ydata)

    def _on_release(self, event):
        self._drag = None

    def _on_motion(self, event):
        if self._drag is None or event.inaxes != self.ax:
            return
        x, y = event.xdata, event.ydata
        snap = event.key is not None and "shift" in event.key
        if snap:
            x, y = round(x / 0.05) * 0.05, round(y / 0.05) * 0.05
        if self._drag == "target":
            self.target = SE2([x, y], self.target.rotation)
        else:
            g = self.target.translation
            phi = np.arctan2(y - g[1], x - g[0])
            if snap:
                phi = np.deg2rad(round(np.rad2deg(phi) / 15) * 15)
            self.target = SE2(g, float(phi))
        self.update()

    def _on_key(self, event):
        if self.on_key is not None:
            self.on_key(self, event.key)
