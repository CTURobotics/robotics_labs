#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2026-11-09
#     Author: Martin Cifka <martin.cifka@cvut.cz>
#
# A matplotlib window with a draggable target frame for the interactive IK scripts
# 03_numerical_ik_interactive.py and 04_analytical_ik_interactive.py. Drag the black dot
# to move the target, drag the ring in front of it to rotate it; hold Shift to snap.
# The script supplies the `on_update` callback that solves the IK and updates the robot
# artists (RobotArtist, TrailArtist); all artists are created once and updated in place,
# which keeps the redraw fast.
from __future__ import annotations

from copy import deepcopy
from typing import Callable

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.widgets import CheckButtons

from robotics_toolbox.core import SE2
from robotics_toolbox.render.planar_manipulator_renderer import (
    PlanarManipulatorRenderer,
)
from robotics_toolbox.render.se2_renderer import SE2Renderer
from robotics_toolbox.robots import PlanarManipulator


class RobotArtist:
    """One robot configuration drawn with the toolbox renderer, updated in place."""

    def __init__(
        self, ax, robot: PlanarManipulator, color="k", lw=3.0, alpha=1.0, label=None
    ):
        self.ax = ax
        self.robot = deepcopy(robot)
        self.lw, self.alpha, self.label = lw, alpha, label
        self._create(color)

    def _create(self, color):
        self.rend = PlanarManipulatorRenderer(
            self.ax, self.robot, color=color, lw=self.lw
        )
        for o in self.artists():
            o.set_alpha(self.alpha)
        if self.label is not None:
            self.handle.set_label(self.label)

    def artists(self):
        return [o for objs in self.rend.plt_objects.values() for o in objs]

    @property
    def handle(self):
        """Artist to use in a legend."""
        return self.rend.plt_objects["links"][0]

    def set(self, q, color=None, structure=None):
        """Show configuration q; optionally change colour or the joint structure."""
        if structure is not None and "".join(structure) != "".join(
            self.robot.structure
        ):
            color = color or self.handle.get_color()
            for o in self.artists():
                o.remove()
            self.robot.structure = structure  # joint markers depend on the structure
            self._create(color)
        self.robot.q = np.asarray(q, float)
        self.rend.update()
        for o in self.artists():
            if color is not None:
                o.set_color(color)
            o.set_visible(True)

    def set_visible(self, visible: bool):
        for o in self.artists():
            o.set_visible(visible)


class TrailArtist:
    """Many configurations of one robot drawn as a single line collection (links and
    gripper) plus one scatter for the joints: cheap even for dozens of poses. The poses
    are coloured along a colormap in their order; all are drawn with reduced opacity
    except the first one (the initial configuration), which is fully opaque."""

    def __init__(
        self, ax, robot: PlanarManipulator, cmap="viridis", alpha=0.35, lw=1.0
    ):
        self.robot = deepcopy(robot)
        self.cmap = matplotlib.colormaps[cmap] if isinstance(cmap, str) else cmap
        self.alpha = alpha  # opacity of all poses but the first
        self.lines = LineCollection([], linewidths=lw)
        ax.add_collection(self.lines)
        self.joints = ax.scatter([], [], s=12, zorder=2)

    def set(self, configurations, positions=None, opaque_first=True):
        """Show the configurations; positions are their colormap positions in [0, 1]
        (evenly spread by default). The first pose is opaque if opaque_first."""
        configurations = list(configurations)
        n = len(configurations)
        if positions is None:
            positions = np.linspace(0.0, 1.0, num=n) if n > 1 else np.zeros(n)
        segments, seg_colors, joints, joint_colors = [], [], [], []
        for k, (q, pos) in enumerate(zip(configurations, positions)):
            color = self.cmap(pos)
            color = (*color[:3], 1.0 if (k == 0 and opaque_first) else self.alpha)
            self.robot.q = np.asarray(q, float)
            frames = self.robot.fk_all_links()
            pts = [f.translation for f in frames]
            segs = [(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
            segs += list(self.robot._gripper_lines(frames[-1]))
            segments += segs
            seg_colors += [color] * len(segs)
            joints += pts[:-1]
            joint_colors += [color] * (len(pts) - 1)
        self.lines.set_segments(segments)
        self.lines.set_color(seg_colors)
        self.joints.set_offsets(np.asarray(joints).reshape(-1, 2))
        self.joints.set_color(joint_colors if joint_colors else "none")


class DragTargetWindow:
    """Figure with a draggable target frame (same look as RendererPlanar).

    on_update(app) is called after every change of the target; it should update the
    robot artists and return a status string shown below the plot.
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
        self.on_update: Callable[[DragTargetWindow], str] | None = None
        self.on_key: Callable[[DragTargetWindow, str], None] | None = None

        self.fig, self.ax = plt.subplots(figsize=(8, 9.2))
        self.fig.subplots_adjust(bottom=0.27, top=0.95)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlim(*(np.asarray(xlim) * lim_scale))
        self.ax.set_ylim(*(np.asarray(ylim) * lim_scale))
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.set_title(title)
        self.ax.grid(True, color="0.9", lw=0.8)
        self.ax.set_axisbelow(True)
        self._tol = 0.03 * np.ptp(self.ax.get_xlim())  # grab radius [m]
        self._status = self.fig.text(
            0.08, 0.135, "", family="monospace", fontsize=9.5, va="top"
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
        # static base frame
        if base_pose is not None:
            SE2Renderer(self.ax, base_pose, length=0.2, lw=2)
            self.ax.plot(*base_pose.translation, "s", color="0.3", ms=9, zorder=5)
        # target frame artists, updated in place
        self._frame = SE2Renderer(self.ax, self.target, length=0.25, lw=2.5)
        (self._knob_line,) = self.ax.plot([], [], ":", color="0.6", lw=1)
        (self._knob,) = self.ax.plot([], [], "o", mfc="none", mec="0.3", mew=2, ms=11)
        (self._dot,) = self.ax.plot([], [], "o", color="k", ms=7, zorder=6)

        self._widgets = []  # keep references, otherwise the widgets stop working
        self._drag = None
        self.fig.canvas.mpl_connect("button_press_event", self._on_press)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_motion)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

    # ----------------------------------------------------------------- widgets
    def add_checkbox(
        self, label: str, on_toggle: Callable[[bool], None], checked=False
    ):
        """Checkbox directly below the plot, aligned left (the status text follows
        underneath); on_toggle(checked) is called on every click."""
        n = len(self._widgets)
        ax = self.fig.add_axes([0.06, 0.185 - 0.045 * n, 0.6, 0.045], frameon=False)
        check = CheckButtons(
            ax,
            [label],
            [checked],
            label_props={"fontsize": [11]},
            frame_props={"sizes": [180], "linewidth": [1.5]},
            check_props={"sizes": [180]},
        )
        check.on_clicked(lambda _label: on_toggle(check.get_status()[0]))
        self._widgets.append(check)
        return check

    # ----------------------------------------------------------------- drawing
    def knob(self) -> np.ndarray:
        a = self.target.rotation.angle
        return self.target.translation + self.KNOB_DIST * np.array(
            [np.cos(a), np.sin(a)]
        )

    def _update_target(self):
        self._frame.t = self.target
        self._frame.update()
        g, knob = self.target.translation, self.knob()
        self._knob_line.set_data([g[0], knob[0]], [g[1], knob[1]])
        self._knob.set_data([knob[0]], [knob[1]])
        self._dot.set_data([g[0]], [g[1]])

    def update(self):
        self._update_target()
        status = self.on_update(self) if self.on_update is not None else ""
        self._status.set_text(status)
        self.fig.canvas.draw_idle()

    def refresh(self, status: str):
        """Redraw the target and show a status text without calling on_update."""
        self._update_target()
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
