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
#   x                     solve again from a random initial configuration
#   space                 replay the iterations of the last solve one by one
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
target = SE2(translation=[-1.5, 1.0], rotation=SO2(angle=np.deg2rad(45 + 90)))

app = DragTargetWindow(
    target,
    base_pose=robot.base_pose,
    lim_scale=2.0,
    title="Numerical IK: drag the target",
    help_text="x: solve from a random initial configuration   space: replay the iterations one by one",
)
trail_artist = TrailArtist(app.ax, robot, cmap=TRAIL_CMAP, alpha=TRAIL_ALPHA)
robot_artist = RobotArtist(app.ax, robot, color="k")
fixed_init = {"q": None}  # None: start from the current (previous) configuration


def thin_indices(n_final: int):
    """Indices of at most MAX_TRAIL_POSES trail poses out of a sequence whose final
    configuration (drawn separately) has index n_final, spread uniformly so that the
    iteration step between the last trail pose and the final one equals the step
    between the trail poses. During a replay only the prefix of this fixed subset is
    shown, so the trail grows and keeps its colours."""
    if MAX_TRAIL_POSES <= 0:
        return np.array([], dtype=int)
    if n_final <= MAX_TRAIL_POSES:
        return np.arange(n_final)
    return np.linspace(0, n_final, MAX_TRAIL_POSES + 1).round().astype(int)[:-1]


def show(sequence, k: int, converged, state: str) -> str:
    """Draw configuration sequence[k] as the robot and sequence[:k] as the trail
    (coloured by their position in the whole sequence); build the status text."""
    n_final = max(len(sequence) - 1, 1)
    idx = thin_indices(n_final)
    idx = idx[idx < k]  # trail = poses before the shown one
    trail_artist.set([sequence[i] for i in idx], positions=idx / n_final)
    q = sequence[k]
    final = k == len(sequence) - 1
    robot_artist.set(q, color="tab:red" if (final and converged is False) else "k")
    q_saved, robot.q = robot.q, q  # FK of the shown configuration via the public API
    flange = robot.flange_pose()
    robot.q = q_saved
    err_pos = np.linalg.norm(flange.translation - app.target.translation)
    err_ang = np.rad2deg(
        (flange.rotation.angle - app.target.rotation.angle + np.pi) % (2 * np.pi)
        - np.pi
    )
    q_deg = ", ".join(f"{v:7.1f}" for v in np.rad2deg(np.angle(np.exp(1j * q))))
    return (
        f"{state}\n"
        f"q [deg, wrapped] = [{q_deg}]\n"
        f"position error {err_pos:.2e} m, orientation error {err_ang:.2e} deg"
    )


def solve(app: DragTargetWindow) -> str:
    """Full solve (target moved or checkbox toggled): run the numerical IK one
    iteration at a time from the fixed initial configuration if the checkbox is on,
    otherwise from the current configuration. The whole sequence of configurations is
    recorded so that it can be replayed with the space key."""
    if fixed_init["q"] is not None:
        robot.q = fixed_init["q"].copy()
    sequence, converged, k = [], False, 0
    for k in range(MAX_ITERATIONS):
        sequence.append(robot.q.copy())  # configuration before the k-th update
        converged = robot.ik_numerical(app.target, max_iterations=1)
        if converged:  # the call only confirmed convergence, q did not change
            break
    if not converged:
        sequence.append(robot.q.copy())  # final configuration after the last update
    n_updates = len(sequence) - 1
    last.update(sequence=sequence, converged=converged, k=None)
    start = start_label["text"] or (
        "fixed initial" if fixed_init["q"] is not None else "previous"
    )
    state = (
        f"converged after {n_updates} iteration(s)"
        if converged
        else f"NOT converged after {n_updates} iteration(s)"
    )
    return show(
        sequence,
        len(sequence) - 1,
        converged,
        f"{state}, started from the {start} configuration",
    )


def replay_step():
    """Space: show the recorded iterations of the last solve one by one (the robot
    configuration itself stays the final one); after the last one start over."""
    seq = last["sequence"]
    if len(seq) < 2:
        app.refresh(
            "nothing to replay (no iteration was needed): drag the target first"
        )
        return
    k = 0 if last["k"] is None or last["k"] >= len(seq) - 1 else last["k"] + 1
    last["k"] = k
    n = len(seq) - 1
    state = f"replay: {'initial configuration' if k == 0 else f'after iteration {k} of {n}'}"
    if k == n:
        state += ", final configuration" + (
            " (converged)" if last["converged"] else " (not converged)"
        )
    app.refresh(show(seq, k, last["converged"], state))


def on_toggle_fixed_init(checked: bool):
    fixed_init["q"] = robot.q.copy() if checked else None
    app.update()


def on_key(app: DragTargetWindow, key: str):
    if key == "x":  # solve the current target from a random initial configuration
        robot.q = robot.sample_configuration()
        if fixed_init["q"] is not None:
            fixed_init["q"] = robot.q.copy()  # the random one becomes the fixed one
        start_label["text"] = "random"
        app.update()
        start_label["text"] = None
    elif key == " ":
        replay_step()


start_label = {"text": None}  # how the start of the current solve is described
last = {
    "sequence": [],
    "converged": None,
    "k": None,
}  # recorded iterations of the last solve


app.on_update = solve
app.on_key = on_key
fixed_init_box = app.add_checkbox("fix initial configuration", on_toggle_fixed_init)

if __name__ == "__main__":
    app.show()
