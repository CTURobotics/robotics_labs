#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from __future__ import annotations
from itertools import islice

import numpy as np
from numpy.typing import ArrayLike
from shapely.geometry import Point, LineString


def nullspace(A, atol=1e-13, rtol=0.0):
    """Compute kernel, i.e. nullspace of the given matrix A."""
    A = np.atleast_2d(A)
    u, s, vh = np.linalg.svd(A)
    tol = max(atol, rtol * s[0])
    nnz = (s >= tol).sum()
    ns = vh[nnz:].conj().T
    return ns


def circle_circle_intersection(
    c0: ArrayLike, r0: float, c1: ArrayLike, r1: float
) -> list[np.ndarray]:
    """Computes intersection of the circles defined by center c_i and radius r_i.
    Returns empty array if there is no solution, two solutions otherwise. If there
    are infinite number of solutions, select two (almost) randomly.
    """
    ca = Point(c0).buffer(r0, quad_segs=64).boundary
    cb = Point(c1).buffer(r1, quad_segs=64).boundary
    ints = ca.intersection(cb)
    if ints.is_empty:
        return []
    return [g.coords[0] for g in islice(ints.geoms, 2)]


def circle_line_intersection(c0: ArrayLike, r0: float, a1: ArrayLike, b1: ArrayLike):
    """Compute intersection of circle (c0, ro) with line segment defined by two
    end-points (a1, b1)"""
    ca = Point(c0).buffer(r0, quad_segs=64).boundary
    lb = LineString([a1, b1])
    ints = ca.intersection(lb)
    if ints.is_empty:
        return []
    return [g.coords[0] for g in ints.geoms]
