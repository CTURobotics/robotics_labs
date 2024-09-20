#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-12-2
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

import numpy as np
from numpy.typing import ArrayLike
from robotics_toolbox.robots import PlanarManipulator


class PlanarManipulatorDynamics(PlanarManipulator):
    def __init__(self, masses=None, *args, **kwargs):
        """Create planar manipulator with dynamics functionality, inherit kinematics
        properties from PlanarManipulator.

        It is assumed that masses are located at the end of the link.
        The equation of motion is given by:
            tau = M(q) ddq + h(q,dq) + damping * dq,
            M(q) is mass matrix implemented in function mass_matrix(q)
            h(q,dq) is implemented in function h(q,dq)

        Forward dynamics is implemented in forward_dynamics(q, dq, tau, damping)
        Inverse dynamics is implemented in inverse_dynamics(q, dq, ddq, damping)
        Constrained forward dynamics is implemented in
        constrained_forward_dynamics(q, dq, tau, damping); the constraint
        enforces gripper to move only on a line that is 45deg w.r.t. x-axis

        It is required to implement/use functions mass_matrix, and h as they can
        be used for forward and inverse dynamics.

        Args:
            masses: masses of links, if None, all links have the same mass of 1.0 kg
            *args: arguments for PlanarManipulator
            **kwargs: keyword arguments for PlanarManipulator
        """
        super().__init__(*args, **kwargs)
        self._g = 9.81  # gravity constant
        self.masses = np.ones(self.dof) * 1.0 if masses is None else np.asarray(masses)

    def mass_matrix(self, q: ArrayLike) -> np.ndarray:
        """Mass matrix of the robot at configuration q."""
        q = np.asarray(q)
        mass_matrix = np.eye(2)
        # todo HW11opt: implement computation of mass matrix
        return mass_matrix

    def h(self, q: ArrayLike, dq: ArrayLike) -> np.ndarray:
        """Coriolis and gravity terms at configuration q and velocity dq."""
        q = np.asarray(q)
        dq = np.asarray(dq)
        h = np.zeros(2)
        # todo HW11opt: implement computation of h
        return h

    def forward_dynamics(
        self, q: ArrayLike, dq: ArrayLike, tau: ArrayLike, damping: float = 0.0
    ) -> np.ndarray:
        """Implement forward dynamics of the robot. I.e. compute ddq from q, dq, tau
        and damping. Use eq. of motion: tau = M(q) ddq + h(q,dq) + damping * dq.
        """
        q = np.asarray(q)
        dq = np.asarray(dq)
        tau = np.asarray(tau)
        ddq = np.zeros_like(q)
        # todo HW11opt: implement unconstrained forward dynamics
        return ddq

    def inverse_dynamics(
        self, q: ArrayLike, dq: ArrayLike, ddq: ArrayLike, damping: float = 0.0
    ) -> np.ndarray:
        """Implement inverse dynamics of the robot. I.e. compute tai from q, dq, ddq,
        and damping. Use eq. of motion: tau = M(q) ddq + h(q,dq) + damping * dq.
        """
        q = np.asarray(q)
        dq = np.asarray(dq)
        ddq = np.asarray(ddq)
        tau = np.zeros_like(q)
        # todo HW11opt: implement unconstrained inverse dynamics
        return tau

    def constrained_forward_dynamics(
        self, q: ArrayLike, dq: ArrayLike, tau: ArrayLike, damping: float = 0.0
    ) -> np.ndarray:
        """Implement constrained forward dynamics of the robot. I.e. compute ddq from q,
        dq, tau and damping. Use eq. of motion:
        tau = M(q) ddq + h(q,dq) + damping * dq + A^T lambda.
        The constraint is fixed, such that end effector moves along line with angle
        45deg w.r.t. x-axis of reference frame.
        """
        q = np.asarray(q)
        dq = np.asarray(dq)
        tau = np.asarray(tau)
        ddq = np.zeros_like(q)
        # todo HW11opt: implement unconstrained forward dynamics
        return ddq
