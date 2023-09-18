#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-4
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

from __future__ import annotations
import numpy as np
from numpy.typing import ArrayLike

from robotics_toolbox.core import SO2


class SE2:
    """Transformation in 2D that is composed of rotation and translation."""

    def __init__(
        self, translation: ArrayLike | None = None, rotation: SO2 | None = None
    ) -> None:
        """Crete an SE2 transformation. Identity is the default."""
        super().__init__()
        self.translation = (
            np.asarray(translation) if translation is not None else np.zeros(2)
        )
        self.rotation = rotation if rotation is not None else SO2()
        assert self.translation.shape == (2,)

    def __mul__(self, other: SE2) -> SE2:
        """Compose two transformation, i.e., self * other"""
        # todo: HW01: implement composition of two transformation.
        pass

    def inverse(self) -> SE2:
        """Compute inverse of the transformation"""
        # todo: HW1 implement inverse
        return SE2()

    def act(self, vector: ArrayLike) -> np.ndarray:
        """Rotate given 2D vector by this SO2 transformation."""
        v = np.asarray(vector)
        assert v.shape == (2,)
        # todo: HW1 implement transformation of a given vector
        return v

    def set_from(self, other: SE2):
        """Copy the properties into current instance."""
        self.translation = other.translation
        self.rotation = other.rotation

    def __eq__(self, other: SE2) -> bool:
        """Returns true if two transformations are almost equal."""
        return (
            np.allclose(self.translation, other.translation)
            and self.rotation == other.rotation
        )

    def __hash__(self):
        return id(self)

    def __repr__(self):
        return (
            f"SE2(translation={self.translation}, rotation=SO2({self.rotation.angle}))"
        )
