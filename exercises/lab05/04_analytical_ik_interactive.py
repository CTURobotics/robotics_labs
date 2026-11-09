#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2026-11-09
#     Author: Martin Cifka <martin.cifka@cvut.cz>
#
# Interactive version of 02_analytical_ik.py: drag the target frame and see all
# solutions of the analytical IK (PlanarManipulator.ik_analytical, your HW05
# implementation) drawn in different colours.
#
#   drag the dot / ring   move / rotate the target
#   1 / 2                 switch the structure RRR / PRR
import warnings

import matplotlib
import numpy as np

from robotics_toolbox.core import SE2
from robotics_toolbox.robots import PlanarManipulator

from ik_drag_window import DragTargetWindow, draw_robot_config

STRUCTURES = {"1": "RRR", "2": "PRR"}
COLORS = matplotlib.colormaps["tab10"].colors

robot = PlanarManipulator(
    link_parameters=[0.5] * 3,
    base_pose=SE2([-0.5, -0.75]),
    structure="PRR",
)
target = robot.flange_pose()


def solve(app: DragTargetWindow) -> str:
    # ik_analytical must return a (possibly empty) list of finite configurations. Report
    # anything else (NaN values, warnings, exceptions) in red instead of hiding it.
    problems = []
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        try:
            solutions = list(robot.ik_analytical(app.target))
        except Exception as e:  # noqa: BLE001 - show the student's error in the window
            solutions = []
            problems.append(f"ik_analytical raised {type(e).__name__}: {e}")
    problems += [f"warning: {w.message}" for w in caught]
    n_bad = sum(not np.all(np.isfinite(q)) for q in solutions)
    if n_bad:
        problems.append(
            f"ik_analytical returned {n_bad} solution(s) containing NaN. An unreachable "
            "target must give an empty list (02_analytical_ik.py prints 'No solution "
            "found.' in that case)."
        )
    solutions = [q for q in solutions if np.all(np.isfinite(q))]
    if problems:
        app.ax.text(
            0.02,
            0.98,
            "\n".join(problems),
            transform=app.ax.transAxes,
            color="tab:red",
            fontsize=9,
            va="top",
            wrap=True,
        )

    for i, q in enumerate(solutions):
        draw_robot_config(app.ax, robot, q, color=COLORS[i], label=f"solution {i + 1}")
    if solutions:
        app.ax.legend(loc="upper right", fontsize=8)

    lines = [f"{robot.structure}: {len(solutions)} solution(s)"]
    for i, q in enumerate(solutions):
        vals = [
            f"q{j + 1} = "
            + (f"{np.rad2deg(v):7.1f} deg" if s == "R" else f"{v:6.3f} m")
            for j, (s, v) in enumerate(zip(robot.structure, q))
        ]
        lines.append(f"solution {i + 1}:  " + "   ".join(vals))
    return "\n".join(lines)


def on_key(app: DragTargetWindow, key: str):
    if key in STRUCTURES:
        robot.structure = STRUCTURES[key]
        app.update()


app = DragTargetWindow(
    target,
    base_pose=robot.base_pose,
    lim_scale=2.0,
    title="Analytical IK: drag the target",
    help_text="1: RRR   2: PRR",
)
app.on_update = solve
app.on_key = on_key

if __name__ == "__main__":
    app.show()
