#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-23
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from __future__ import annotations
import numpy as np
from numpy.typing import ArrayLike

from robotics_toolbox.core import SE2, SE3, SO3


def distance_between_configurations(
    a: ArrayLike | SE2 | SE3, b: ArrayLike | SE2 | SE3
) -> float:
    """Compute distance between two configurations, expressed either in task-space
    SE2/SE3 or joint space np.ndarray"""
    assert isinstance(a, type(b))
    if isinstance(a, SE2):
        d = a.inverse() * b
        return np.linalg.norm(np.append(d.translation, d.rotation.angle))
    elif isinstance(a, SE3):
        d = a.inverse() * b
        return np.linalg.norm(np.append(d.translation, d.rotation.log()))
    else:
        return np.linalg.norm(a - b)


def interpolate(
    a: ArrayLike | SE2 | SE3, b: ArrayLike | SE2 | SE3, d: float
) -> np.ndarray | SE2 | SE3:
    """Interpolate between two configurations, s.t. dist(a,b) = d"""
    assert isinstance(a, type(b))
    if isinstance(a, SE2):
        nrm = distance_between_configurations(a, b)
        diff = a.inverse() * b
        log_diff = d / nrm * np.append(diff.translation, diff.rotation.angle)
        return a * SE2(log_diff[:2], log_diff[2])
    elif isinstance(a, SE3):
        nrm = distance_between_configurations(a, b)
        diff = a.inverse() * b
        log_diff = d / nrm * np.append(diff.translation, diff.rotation.log())
        return a * SE3(translation=log_diff[:3], rotation=SO3.exp(log_diff[3:]))
    else:
        return a + d * (b - a) / distance_between_configurations(a, b)
