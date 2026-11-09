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
#   checkbox              start every solve from a fixed initial configuration (the
#                         one the robot had when the box was checked) instead of the
#                         previous solution
import numpy as np

from robotics_toolbox.core import SE2, SO2
from robotics_toolbox.robots import PlanarManipulator

from ik_drag_window import DragTargetWindow, RobotArtist, TrailArtist

MAX_ITERATIONS = 50
MAX_TRAIL_POSES = 12  # grey trail: all poses if there are at most this many, otherwise
#                     this many evenly spaced ones (0 = no trail)
TRAIL_CMAP = "plasma_r"  # colormap of the trail, poses coloured in iteration order
TRAIL_ALPHA = 0.35  # opacity of the trail poses except the initial configuration

robot = PlanarManipulator(
    link_parameters=[0.3] * 5,
    base_pose=SE2([-0.75, 0.0]),
    structure="RRRRR",
)
q_init = robot.q.copy()
target = SE2(translation=[-1.5, 1.0], rotation=SO2(angle=np.deg2rad(45 + 90)))

app = DragTargetWindow(
    target,
    base_pose=robot.base_pose,
    lim_scale=2.0,
    title="Numerical IK: drag the target",
    help_text="r: reset configuration   x: random configuration   space: single iteration",
)
trail_artist = TrailArtist(app.ax, robot, cmap=TRAIL_CMAP, alpha=TRAIL_ALPHA)
robot_artist = RobotArtist(app.ax, robot, color="k")
fixed_init = {"q": None}  # None: start from the current (previous) configuration


def solve(app: DragTargetWindow, iterations: int = MAX_ITERATIONS) -> str:
    """Run the numerical IK from the current configuration one iteration at a time;
    show the intermediate configurations in grey and the final one in black."""
    if fixed_init["q"] is not None:
        robot.q = fixed_init["q"].copy()
    trail, converged, k = [], False, 0
    for k in range(iterations):
        trail.append(robot.q.copy())
        converged = robot.ik_numerical(app.target, max_iterations=1)
        if converged:
            break
    # trail: the starting configuration and the intermediate ones. With more than
    # MAX_TRAIL_POSES of them, thin out uniformly over the whole sequence *including*
    # the final configuration (drawn in black, hence dropped from the trail), so that
    # the iteration step between the last trail pose and the final one is the same as
    # between the trail poses.
    if MAX_TRAIL_POSES > 0 and len(trail) > MAX_TRAIL_POSES:
        n_final = len(trail)  # index of the final configuration in the full sequence
        idx = np.linspace(0, n_final, MAX_TRAIL_POSES + 1).round().astype(int)[:-1]
        trail = [trail[i] for i in idx]
    trail_artist.set(trail if MAX_TRAIL_POSES > 0 else [])
    robot_artist.set(robot.q, color="k" if converged else "tab:red")

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
    start = (
        "fix initial configuration"
        if fixed_init["q"] is not None
        else "previous configuration"
    )
    return (
        f"{state}, started from the {start}\n"
        f"q [deg, wrapped] = [{q_deg}]\n"
        f"position error {err_pos:.2e} m, orientation error {err_ang:.2e} deg"
    )


def on_toggle_fixed_init(checked: bool):
    fixed_init["q"] = robot.q.copy() if checked else None
    app.update()


def on_key(app: DragTargetWindow, key: str):
    if key in ("r", "x"):
        robot.q = q_init.copy() if key == "r" else robot.sample_configuration()
        if fixed_init["q"] is not None:
            fixed_init["q"] = (
                robot.q.copy()
            )  # the new configuration becomes the fixed one
    elif key == " ":
        app.on_update = lambda a: solve(a, iterations=1)
        app.update()
        app.on_update = solve
        return
    else:
        return
    app.update()


app.on_update = solve
app.on_key = on_key
fixed_init_box = app.add_checkbox("fix initial configuration", on_toggle_fixed_init)

if __name__ == "__main__":
    app.show()
