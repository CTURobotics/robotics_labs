#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2026-09-22
#     Author: Martin Cifka <martin.cifka@cvut.cz>
#
# Matplotlib drawing helpers for the SE(2) frames example (04_se2_frames_example.py and
# the notebook of the same name). Nothing here is important for the math.
from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arc

from robotics_toolbox.core import SE2, SO2

AXIS_LEN = 1.0
# Axes and their labels are always drawn x = red, y = green; frame names and origins are black.
# The frame colors are used only for the vectors/projections expressed in that frame. The
# point P has its own color and the parts of the transformation (translation arrow, angle
# arc) are black.
COLORS = {
    "x": "tab:red",
    "y": "tab:green",
    "A": "tab:blue",
    "B": "tab:purple",
    "A'": "k",
    "P": "tab:orange",
}


def new_axes(title: str, xlim=(-2.5, 4.5), ylim=(-2.5, 3.5)):
    """Create axes with a unit grid (the grid is only a drawing aid)."""
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect("equal")
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_xticks(np.arange(np.ceil(xlim[0]), np.floor(xlim[1]) + 1))
    ax.set_yticks(np.arange(np.ceil(ylim[0]), np.floor(ylim[1]) + 1))
    ax.grid(True, color="0.85", lw=0.8)
    ax.tick_params(labelbottom=False, labelleft=False, length=0)
    ax.set_title(title)
    return fig, ax


def draw_frame(ax, pose: SE2, name: str, color="k", lw=2.5, **kw):
    """Draw a coordinate frame whose pose on the grid is given by SE2 pose.
    Axes and their labels are red (x) and green (y); the frame name and origin are black.
    """
    o = pose.translation
    for i, axis_name in enumerate(("x", "y")):
        d = pose.rotation.rot[:, i] * AXIS_LEN
        ax.annotate(
            "",
            xy=o + d,
            xytext=o,
            arrowprops=dict(arrowstyle="->", color=COLORS[axis_name], lw=lw, **kw),
        )
        ax.text(
            *(o + 1.15 * d),
            f"${axis_name}_{{{name}}}$",
            color=COLORS[axis_name],
            ha="center",
            va="center",
        )
    ax.plot(*o, "o", color=color, ms=5)
    # put the frame name on the side opposite to the axes so that names do not collide
    away = -pose.rotation.rot.sum(axis=1)
    away = 0.3 * away / np.linalg.norm(away)
    ax.text(
        *(o + away),
        f"${name}$",
        color=color,
        fontsize=13,
        fontweight="bold",
        ha="center",
        va="center",
    )


def draw_point(ax, p, name: str = "P", color=COLORS["P"]):
    ax.plot(*p, "o", color=color, ms=9, zorder=5)
    ax.text(
        p[0] + 0.15,
        p[1] + 0.1,
        f"${name}$",
        color=color,
        fontsize=13,
        fontweight="bold",
    )


def draw_vector(ax, start, end, color="k", label=None, ls="-", lw=1.2):
    """Thin arrow between two points (only the frame axes are drawn bold)."""
    ax.annotate(
        "",
        xy=end,
        xytext=start,
        arrowprops=dict(
            arrowstyle="->", color=color, lw=lw, ls=ls, shrinkA=0, shrinkB=0
        ),
    )
    if label:
        mid = (np.asarray(start) + np.asarray(end)) / 2
        ax.text(*(mid + 0.12), label, color=color, fontsize=11)


def draw_projection(ax, pose: SE2, p_F, color):
    """Dashed lines showing coordinates of a point p_F along the axes of frame F."""
    o = pose.translation
    along_x = pose.act([p_F[0], 0.0])
    p_grid = pose.act(p_F)
    ax.plot([o[0], along_x[0]], [o[1], along_x[1]], "--", color=color, lw=1.2)
    ax.plot([along_x[0], p_grid[0]], [along_x[1], p_grid[1]], "--", color=color, lw=1.2)


def draw_angle(ax, center, angle_from_deg, angle_to_deg, radius, color, label):
    """Arc from angle_from to angle_to (counter-clockwise) with a label."""
    a0, a1 = angle_from_deg, angle_to_deg
    if a1 < a0:
        a1 += 360
    ax.add_patch(
        Arc(center, 2 * radius, 2 * radius, theta1=a0, theta2=a1, color=color, lw=1.5)
    )
    mid = np.deg2rad((a0 + a1) / 2)
    ax.text(
        *(center + 1.3 * radius * np.array([np.cos(mid), np.sin(mid)])),
        label,
        color=color,
        fontsize=11,
        ha="center",
        va="center",
    )


def add_text(ax, lines):
    """Notes printed below the plot; the figure is enlarged to make room for them."""
    ax.figure.set_size_inches(7, 8.5)
    ax.figure.subplots_adjust(bottom=0.27)
    ax.figure.text(
        0.08,
        0.02,
        "\n".join(lines),
        va="bottom",
        ha="left",
        fontsize=10.5,
        family="monospace",
        bbox=dict(boxstyle="round", fc="white", ec="0.7"),
    )


def fmt(M) -> str:
    """Pretty print an array; tiny values are printed as 0 instead of -0."""
    M = np.asarray(M)
    M = np.where(np.abs(M) < 1e-9, 0.0, M)
    return np.array2string(M, precision=3, suppress_small=True)


def deg(t: SE2 | SO2) -> float:
    """Rotation angle of a transformation in degrees, in the range [0, 360)."""
    rot = t.rotation if isinstance(t, SE2) else t
    return np.rad2deg(rot.angle) % 360
