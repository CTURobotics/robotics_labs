#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-19
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
# Test utilities

from __future__ import annotations
import numpy as np
import pinocchio as pin
from pinocchio import OP_FRAME

from robotics_toolbox.core import SE2, SO2
from robotics_toolbox.robots import PlanarManipulator


def se2_to_pin_se3(pose: SE2) -> pin.SE3:
    """Create pin.SE3 transformation from SE2 with (tx,ty,0) and Rz(angle)."""
    return pin.SE3(
        pin.exp(np.asarray([0, 0, pose.rotation.angle])),
        np.pad(pose.translation, (0, 1)),
    )


def assert_se2_equals_pin_se3(self, a: SE2, b: pin.SE3):
    m = b.homogeneous
    self.assertTrue(np.allclose(m[:2, :2], a.rotation.rot))
    self.assertTrue(np.allclose(m[:2, 3], a.translation))


def planar_manipulator_to_pin(robot: PlanarManipulator) -> pin.Model:
    model: pin.Model = pin.Model()
    jid = 0
    pose = robot.base_pose
    for li, qi, jtype in zip(robot.link_parameters, robot.q, robot.structure):
        if jtype == "R":
            jid = model.addJoint(
                jid, pin.JointModelRZ(), se2_to_pin_se3(pose), f"j{jid}"
            )
            pose = SE2(translation=[li, 0])
        elif jtype == "P":
            pose *= SE2(rotation=SO2(li))
            jid = model.addJoint(
                jid, pin.JointModelPX(), se2_to_pin_se3(pose), f"j{jid}"
            )
            pose = SE2()

    model.addFrame(pin.Frame("flange", jid, 0, se2_to_pin_se3(pose), OP_FRAME))
    return model


def sample_planar_manipulator(n: int | None = None):
    if n is None:
        n = np.random.randint(1, 5)
    return PlanarManipulator(
        link_parameters=np.random.uniform(0.1, 0.3, size=n),
        base_pose=SE2(
            translation=np.random.uniform(-0.5, 0.5, size=2),
            rotation=SO2(np.random.uniform(-np.pi, np.pi)),
        ),
        structure=np.random.choice(["R", "P"], size=n),
    )
