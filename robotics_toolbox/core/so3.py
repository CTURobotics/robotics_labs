#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-4
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from __future__ import annotations
import numpy as np
from numpy.typing import ArrayLike


class SO3:
    """This class represents an SO3 rotations internally represented by rotation
    matrix."""

    def __init__(self, rotation_matrix: ArrayLike | None = None) -> None:
        """Creates a rotation transformation from rot_vector."""
        super().__init__()
        self.rot: np.ndarray = (
            np.asarray(rotation_matrix) if rotation_matrix is not None else np.eye(3)
        )

    @staticmethod
    def exp(rot_vector: ArrayLike) -> SO3:
        """Compute SO3 transformation from a given rotation vector, i.e. exponential
        representation of the rotation."""
        v = np.asarray(rot_vector)
        assert v.shape == (3,)
        t = SO3()
        # todo HW01: implement Rodrigues' formula, t.rot = ...
        return t

    def log(self) -> np.ndarray:
        """Compute rotation vector from this SO3"""
        # todo HW01: implement computation of rotation vector from this SO3
        v = np.zeros(3)
        return v

    def __mul__(self, other: SO3) -> SO3:
        """Compose two rotations, i.e., self * other"""
        # todo: HW01: implement composition of two rotation.
        return SO3()

    def inverse(self) -> SO3:
        """Return inverse of the transformation."""
        # todo: HW01: implement inverse, do not use np.linalg.inverse()
        return SO3()

    def act(self, vector: ArrayLike) -> np.ndarray:
        """Rotate given vector by this transformation."""
        v = np.asarray(vector)
        assert v.shape == (3,)
        return self.rot @ v

    def __eq__(self, other: SO3) -> bool:
        """Returns true if two transformations are almost equal."""
        return np.allclose(self.rot, other.rot)

    @staticmethod
    def rx(angle: float) -> SO3:
        """Return rotation matrix around x axis."""
        # todo: HW1opt: implement rx
        raise NotImplementedError("RX needs to be implemented.")

    @staticmethod
    def ry(angle: float) -> SO3:
        """Return rotation matrix around y axis."""
        # todo: HW1opt: implement ry
        raise NotImplementedError("RY needs to be implemented.")

    @staticmethod
    def rz(angle: float) -> SO3:
        """Return rotation matrix around z axis."""
        # todo: HW1opt: implement rz
        raise NotImplementedError("RZ needs to be implemented.")

    @staticmethod
    def from_quaternion(q: ArrayLike) -> SO3:
        """Compute rotation from quaternion in a form [qx, qy, qz, qw]."""
        # todo: HW1opt: implement from quaternion
        raise NotImplementedError("From quaternion needs to be implemented")

    def to_quaternion(self) -> np.ndarray:
        """Compute quaternion from self."""
        # todo: HW1opt: implement to quaternion
        raise NotImplementedError("To quaternion needs to be implemented")

    @staticmethod
    def from_angle_axis(angle: float, axis: ArrayLike) -> SO3:
        """Compute rotation from angle axis representation."""
        # todo: HW1opt: implement from angle axis
        raise NotImplementedError("Needs to be implemented")

    def to_angle_axis(self) -> tuple[float, np.ndarray]:
        """Compute angle axis representation from self."""
        # todo: HW1opt: implement to angle axis
        raise NotImplementedError("Needs to be implemented")

    @staticmethod
    def from_euler_angles(angles: ArrayLike, seq: list[str]) -> SO3:
        """Compute rotation from euler angles defined by a given sequence.
        angles: is a three-dimensional array of angles
        seq: is a list of axis around which angles rotate, e.g. 'xyz', 'xzx', etc.
        """
        # todo: HW1opt: implement from euler angles
        raise NotImplementedError("Needs to be implemented")

    def __hash__(self):
        return id(self)
