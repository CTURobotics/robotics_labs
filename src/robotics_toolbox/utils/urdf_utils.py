#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2026-09-10
#     Author: Martin Cífka <martin.cifka@cvut.cz>
#

"""Utilities for working with URDF models loaded by yourdfpy. This module is provided
code used by the spatial manipulator, the renderer and the tests; there is nothing to
implement here."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yourdfpy

from robotics_toolbox.core import SE3, SO3

ACTUATED_JOINT_TYPES = ("revolute", "continuous", "prismatic")


def chain_ordered_joints(urdf: yourdfpy.URDF) -> list[yourdfpy.Joint]:
    """Return actuated joints (i.e. not fixed and not mimicking another joint) ordered
    by the traversal of the kinematic tree from the base link. The order of the joints
    in the URDF file is therefore irrelevant, only the structure matters. This is the
    order in which configuration vectors are interpreted."""
    joints_by_parent: dict[str, list[yourdfpy.Joint]] = {}
    for joint in urdf.robot.joints:
        joints_by_parent.setdefault(joint.parent, []).append(joint)

    ordered = []
    queue = [urdf.base_link]
    while queue:
        link = queue.pop(0)
        for joint in joints_by_parent.get(link, []):
            if joint.type in ACTUATED_JOINT_TYPES and joint.mimic is None:
                ordered.append(joint)
            queue.append(joint.child)
    return ordered


def chain_ordered_joint_names(urdf: yourdfpy.URDF) -> list[str]:
    """Names of the actuated joints, see chain_ordered_joints."""
    return [joint.name for joint in chain_ordered_joints(urdf)]


def leaf_links(urdf: yourdfpy.URDF) -> list[str]:
    """Return links that have no child, i.e. the ends of the kinematic chains."""
    parents = {joint.parent for joint in urdf.robot.joints}
    return [name for name in urdf.link_map if name not in parents]


def leaf_link(urdf: yourdfpy.URDF) -> str:
    """Return the single end link of a serial kinematic chain."""
    leaves = leaf_links(urdf)
    assert len(leaves) == 1, f"Expected a single end link, found {leaves}."
    return leaves[0]


def ancestor_joint_names(urdf: yourdfpy.URDF, link_name: str) -> set[str]:
    """Return names of all joints on the path from the base link to the given link."""
    joint_by_child = {joint.child: joint for joint in urdf.robot.joints}
    ancestors = set()
    while link_name in joint_by_child:
        joint = joint_by_child[link_name]
        ancestors.add(joint.name)
        link_name = joint.parent
    return ancestors


def joint_limits(joint: yourdfpy.Joint) -> tuple[float, float]:
    """Return (lower, upper) limits of the joint, [-pi, pi] if not specified."""
    limit = joint.limit
    if (
        joint.type == "continuous"
        or limit is None
        or limit.lower is None
        or limit.upper is None
    ):
        return -np.pi, np.pi
    return float(limit.lower), float(limit.upper)


def joint_axis(joint: yourdfpy.Joint) -> np.ndarray:
    """Return the joint axis in the joint frame, URDF default is x-axis."""
    if joint.axis is None:
        return np.array([1.0, 0.0, 0.0])
    return np.asarray(joint.axis, dtype=float)


def srdf_disabled_collision_pairs(srdf_path: Path | str) -> set[frozenset[str]]:
    """Parse <disable_collisions link1="" link2=""/> elements from an SRDF file and
    return the set of link pairs for which collision checking is disabled."""
    root = ET.parse(str(srdf_path)).getroot()
    return {
        frozenset((e.get("link1"), e.get("link2")))
        for e in root.iter("disable_collisions")
    }


def hom_to_se3(t: np.ndarray) -> SE3:
    """Convert 4x4 homogeneous matrix to SE3."""
    t = np.asarray(t)
    return SE3(translation=t[:3, 3], rotation=SO3(rotation_matrix=t[:3, :3]))
