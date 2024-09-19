#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike


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
    c0 = np.asarray(c0)
    c1 = np.asarray(c1)
    d = np.linalg.norm(c1 - c0)
    if r0 + r1 < d < np.abs(r0 - r1):
        return []

    if np.isclose(d, 0) and np.isclose(r0, r1):
        return [
            c0 + [r0 * np.cos(a), r0 * np.sin(a)]
            for a in np.random.uniform(-np.pi, np.pi, 2)
        ]
    a = (r0**2 - r1**2 + d**2) / (2 * d)
    h = np.sqrt(r0**2 - a**2)
    x0, y0 = c0
    x1, y1 = c1
    x2 = x0 + a * (x1 - x0) / d
    y2 = y0 + a * (y1 - y0) / d
    x3 = x2 + h * (y1 - y0) / d
    y3 = y2 - h * (x1 - x0) / d
    x4 = x2 - h * (y1 - y0) / d
    y4 = y2 + h * (x1 - x0) / d

    return [np.array([x3, y3]), np.array([x4, y4])]


def circle_line_intersection(
    c: ArrayLike, r: float, a: ArrayLike, b: ArrayLike
) -> list[np.ndarray]:
    """Compute intersection of circle (c, r) with line defined by two points (a, b)"""
    c = np.asarray(c)
    a = np.asarray(a) - c
    b = np.asarray(b) - c
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dr = np.sqrt(dx**2 + dy**2)
    d = a[0] * b[1] - b[0] * a[1]

    discriminant = r**2 * dr**2 - d**2
    if discriminant < 0:
        return []
    elif np.isclose(discriminant, 0):
        x = (d * dy + np.sign(dy) * dx * np.sqrt(discriminant)) / (dr**2)
        y = (-d * dx + np.abs(dy) * np.sqrt(discriminant)) / (dr**2)
        return [np.array([x, y]) + c]
    else:
        sols = []
        for pm in [-1, 1]:
            sgn_dy = 1 if dy > 0 else -1
            x = (d * dy + pm * sgn_dy * dx * np.sqrt(discriminant)) / (dr**2)
            y = (-d * dx + pm * np.abs(dy) * np.sqrt(discriminant)) / (dr**2)
            sols.append(np.array([x, y]) + c)
        return sols
