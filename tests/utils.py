#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-19
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
# Test utilities

from __future__ import annotations

import inspect
import io
import re
import unittest

import numpy as np
from numpy.typing import ArrayLike
import yourdfpy
from scipy.optimize import approx_fprime
from scipy.spatial.transform import Rotation

from robotics_toolbox.core import SE2, SO2
from robotics_toolbox.robots import PlanarManipulator

"""=== Import restrictions for the student files ==="""

# Students have to implement everything with python standard library and numpy only.
# Every module listed here provides a shortcut for some homework (rotations, kinematics,
# URDF loading, ...) and therefore must not appear in the student files.
FORBIDDEN_MODULES = (
    "pinocchio",
    "scipy",
    "cv2",
    "yourdfpy",
    "viser",
    "trimesh",
    "robot_descriptions",
    "fcl",
    "sympy",
    "transforms3d",
    "spatialmath",
    # provided code of the toolbox that computes kinematics from URDF
    "spatial_manipulator",
    "SpatialManipulator",
    "urdf_utils",
)

# The reference implementations of the tests must not be imported by the student code.
FORBIDDEN_PATTERNS = (r"\btests\.", r"\bfrom\s+tests\b", r"\bimport\s+tests\b")


def assert_no_forbidden_imports(
    testcase: unittest.TestCase, obj, allow: tuple[str, ...] = ()
):
    """Assert that the source file defining @param obj does not mention any of the
    FORBIDDEN_MODULES (except the ones explicitly listed in @param allow)."""
    with open(inspect.getfile(obj)) as f:
        source = f.read()
    for module in FORBIDDEN_MODULES:
        if module in allow:
            continue
        testcase.assertFalse(
            re.search(rf"\b{re.escape(module)}\b", source),
            msg=f"Module '{module}' must not be used in {inspect.getfile(obj)}.",
        )
    for pattern in FORBIDDEN_PATTERNS:
        testcase.assertFalse(
            re.search(pattern, source),
            msg=f"The tests package must not be imported in {inspect.getfile(obj)}.",
        )


"""=== Reference rotations and transformations (oracle for hw01) ==="""


def exp3(rot_vector: ArrayLike) -> np.ndarray:
    """Reference rotation matrix computed from rotation vector."""
    return Rotation.from_rotvec(np.asarray(rot_vector, dtype=float)).as_matrix()


def log3(rot: ArrayLike) -> np.ndarray:
    """Reference rotation vector computed from rotation matrix."""
    return Rotation.from_matrix(np.asarray(rot, dtype=float)).as_rotvec()


def quat_xyzw(rot: ArrayLike) -> np.ndarray:
    """Reference quaternion [qx, qy, qz, qw] computed from rotation matrix."""
    return Rotation.from_matrix(np.asarray(rot, dtype=float)).as_quat()


def rot_from_quat_xyzw(q: ArrayLike) -> np.ndarray:
    """Reference rotation matrix computed from quaternion [qx, qy, qz, qw]."""
    return Rotation.from_quat(np.asarray(q, dtype=float)).as_matrix()


def rot2(angle: float) -> np.ndarray:
    """Reference 2D rotation matrix."""
    return Rotation.from_euler("z", angle).as_matrix()[:2, :2]


def angle2(rot: ArrayLike) -> float:
    """Reference angle [rad] of a 2D rotation matrix."""
    rot3 = np.eye(3)
    rot3[:2, :2] = np.asarray(rot, dtype=float)
    return float(Rotation.from_matrix(rot3).as_rotvec()[2])


def hom2(angle: float = 0.0, translation: ArrayLike = (0.0, 0.0)) -> np.ndarray:
    """Reference 3x3 homogeneous matrix of a planar transformation."""
    h = np.eye(3)
    h[:2, :2] = rot2(angle)
    h[:2, 2] = translation
    return h


def hom3(rot: ArrayLike, translation: ArrayLike) -> np.ndarray:
    """Reference 4x4 homogeneous matrix from rotation matrix and translation."""
    h = np.eye(4)
    h[:3, :3] = rot
    h[:3, 3] = translation
    return h


def hom3_to_hom2(h: np.ndarray) -> np.ndarray:
    """Planar (3x3) homogeneous matrix of a spatial (4x4) one, i.e. x, y and yaw."""
    out = np.eye(3)
    out[:2, :2] = h[:2, :2]
    out[:2, 2] = h[:2, 3]
    return out


def assert_se2_equals_hom(testcase: unittest.TestCase, a: SE2, h: np.ndarray):
    """Assert that SE2 transformation equals the 3x3 homogeneous matrix."""
    testcase.assertTrue(np.allclose(h[:2, :2], a.rotation.rot))
    testcase.assertTrue(np.allclose(h[:2, 2], a.translation))


"""=== Reference planar manipulator kinematics (oracle for hw02 and hw04) ===

The planar manipulator is described as a URDF and the kinematics is computed by yourdfpy
(forward kinematics) and scipy (numerical differentiation); see the constructor of
PlanarManipulator for the definition of the kinematic chain and the lab02 documentation
for the definition of the link frames.
"""


def planar_manipulator_urdf(robot: PlanarManipulator) -> str:
    """URDF description of the planar manipulator. Link 'world' is the reference frame,
    'base_link' is placed at the base pose, revolute joints rotate about z, prismatic
    joints translate along x, and fixed link 'tip_i' is the frame attached to the
    i-th link (its x-axis points along the link and its origin is at the link's end)."""
    rot = robot.base_pose.rotation.rot
    assert np.allclose(rot.T @ rot, np.eye(2)) and np.isclose(
        np.linalg.det(rot), 1.0
    ), "Rotation of the base pose is not a valid rotation, implement SO2 (HW01) first."
    tx, ty = robot.base_pose.translation
    lines = [
        '<robot name="planar_manipulator">',
        '<link name="world"/>',
        '<link name="base_link"/>',
        '<joint name="base_joint" type="fixed"><parent link="world"/>'
        f'<child link="base_link"/><origin xyz="{tx} {ty} 0" rpy="0 0 {angle2(rot)}"/>'
        "</joint>",
    ]
    parent = "base_link"
    for i, (li, jtype) in enumerate(zip(robot.link_parameters, robot.structure)):
        if jtype == "R":
            joint_origin, axis, tip_origin = "0 0 0", "0 0 1", f'xyz="{li} 0 0"'
        elif jtype == "P":
            joint_origin, axis, tip_origin = f"0 0 {li}", "1 0 0", 'xyz="0 0 0"'
        else:
            raise ValueError(f"Unknown joint type {jtype}")
        lines += [
            f'<joint name="joint_{i}" type="{"revolute" if jtype == "R" else "prismatic"}">'
            f'<parent link="{parent}"/><child link="link_{i}"/>'
            f'<origin xyz="0 0 0" rpy="{joint_origin}"/><axis xyz="{axis}"/>'
            '<limit lower="-100" upper="100" effort="1" velocity="1"/></joint>',
            f'<link name="link_{i}"/>',
            f'<joint name="tip_joint_{i}" type="fixed"><parent link="link_{i}"/>'
            f'<child link="tip_{i}"/><origin {tip_origin} rpy="0 0 0"/></joint>',
            f'<link name="tip_{i}"/>',
        ]
        parent = f"tip_{i}"
    lines.append("</robot>")
    return "\n".join(lines)


def _load_planar_manipulator_urdf(robot: PlanarManipulator) -> yourdfpy.URDF:
    return yourdfpy.URDF.load(
        io.BytesIO(planar_manipulator_urdf(robot).encode()),
        load_meshes=False,
        build_scene_graph=True,
    )


def _planar_frames(urdf: yourdfpy.URDF, q: np.ndarray) -> list[np.ndarray]:
    """4x4 frames of the base and all the link tips for the configuration q."""
    urdf.update_cfg({f"joint_{i}": qi for i, qi in enumerate(q)})
    return [urdf.get_transform("base_link", "world")] + [
        urdf.get_transform(f"tip_{i}", "world") for i in range(len(q))
    ]


def planar_fk_reference(robot: PlanarManipulator) -> list[np.ndarray]:
    """Reference forward kinematics of the planar manipulator computed by yourdfpy.
    Returns 3x3 homogeneous matrices of the base frame followed by the frames attached
    to the links; the last frame is the flange."""
    urdf = _load_planar_manipulator_urdf(robot)
    q = np.asarray(robot.q, dtype=float)
    return [hom3_to_hom2(f) for f in _planar_frames(urdf, q)]


def planar_jacobian_reference(robot: PlanarManipulator) -> np.ndarray:
    """Reference 3xDoF jacobian of the flange (vx, vy, wz expressed in the reference
    frame) computed by numerical differentiation (scipy) of the yourdfpy kinematics."""
    urdf = _load_planar_manipulator_urdf(robot)
    q = np.asarray(robot.q, dtype=float)
    rot0 = _planar_frames(urdf, q)[-1][:3, :3]

    def flange(qi: np.ndarray) -> np.ndarray:
        t = _planar_frames(urdf, qi)[-1]
        # the angle is measured w.r.t. the nominal flange rotation to avoid the wrap
        # around +-pi in the numerical differentiation
        yaw = Rotation.from_matrix(rot0.T @ t[:3, :3]).as_rotvec()[2]
        return np.array([t[0, 3], t[1, 3], yaw])

    return approx_fprime(q, flange, 1e-7)


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
