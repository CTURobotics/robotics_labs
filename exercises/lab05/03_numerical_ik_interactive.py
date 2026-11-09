#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2026-11-09
#     Author: Martin Cifka <martin.cifka@cvut.cz>
#
# Interactive version of 01_numerical_ik.py: drag the target frame and watch the
# numerical IK (PlanarManipulator.ik_numerical, your HW05 implementation) follow it.
# The solver always starts from the current configuration, so dragging shows how the
# solution depends on the initial guess. Intermediate iterations are drawn in grey.
#
#   drag the dot / ring   move / rotate the target
#   r                     reset the robot to its initial configuration
#   x                     random initial configuration
#   space                 perform a single iteration
import numpy as np

from robotics_toolbox.core import SE2, SO2
from robotics_toolbox.robots import PlanarManipulator

from ik_drag_window import DragTargetWindow, draw_robot_config

MAX_ITERATIONS = 50

robot = PlanarManipulator(
    link_parameters=[0.3] * 5,
    base_pose=SE2([-0.75, 0.0]),
    structure="RRRRR",
)
q_init = robot.q.copy()
target = SE2(translation=[-1.5, 1.0], rotation=SO2(angle=np.deg2rad(45 + 90)))


def solve(app: DragTargetWindow, iterations: int = MAX_ITERATIONS) -> str:
    """Run the numerical IK from the current configuration one iteration at a time;
    draw the intermediate configurations in grey and the final one in black."""
    trail, converged, k = [], False, 0
    for k in range(iterations):
        trail.append(robot.q.copy())
        converged = robot.ik_numerical(app.target, max_iterations=1)
        if converged:
            break
    for q in trail[1:]:
        draw_robot_config(app.ax, robot, q, color="0.85", lw=1.0, alpha=0.4)
    draw_robot_config(app.ax, robot, robot.q, color="k" if converged else "tab:red")

    flange = robot.flange_pose()
    err_pos = np.linalg.norm(flange.translation - app.target.translation)
    err_ang = np.rad2deg(
        (flange.rotation.angle - app.target.rotation.angle + np.pi) % (2 * np.pi)
        - np.pi
    )
    q_deg = ", ".join(f"{v:7.1f}" for v in np.rad2deg(np.angle(np.exp(1j * robot.q))))
    state = (
        f"converged after {k + 1} iteration(s)"
        if converged
        else f"NOT converged after {iterations} iteration(s)"
    )
    return (
        f"{state}\n"
        f"q [deg, wrapped] = [{q_deg}]\n"
        f"position error {err_pos:.2e} m, orientation error {err_ang:.2e} deg"
    )


def on_key(app: DragTargetWindow, key: str):
    if key == "r":
        robot.q = q_init.copy()
    elif key == "x":
        robot.q = robot.sample_configuration()
    elif key == " ":
        app.on_update = lambda a: solve(a, iterations=1)
        app.update()
        app.on_update = solve
        return
    else:
        return
    app.update()


app = DragTargetWindow(
    target,
    base_pose=robot.base_pose,
    lim_scale=2.0,
    title="Numerical IK: drag the target",
    help_text="r: reset configuration   x: random configuration   space: single iteration",
)
app.on_update = solve
app.on_key = on_key

if __name__ == "__main__":
    app.show()
