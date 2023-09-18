#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-08-21
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import numpy as np
from numpy.typing import ArrayLike

from robotics_toolbox.core import SE2


class PlanarManipulator:
    def __init__(
        self,
        link_lengths: ArrayLike | None = None,
        fixed_rotations: ArrayLike | None = None,
        structure: list[str] | str | None = None,
        base_pose: SE2 | None = None,
        gripper_length: float = 0.2,
    ) -> None:
        super().__init__()
        """
        Creates a planar manipulator composed by rotational and prismatic joints.
        The manipulator kinematics is defines by following kinematics chain:
         T_flange = (T_0 T(q_0)) (T_1 T(q_1)) ... (T_n T(q_n)), 
        where
         T_i describes the pose of the joint w.r.t. the previous frame computed as:
         T_0 = base_pose
         T_i = R(alpha_{i-1}) Tx(l_{i-1})
        with
         alpha_i is taken from @param fixed_rotations, and
         l_i is taken from @param link_lengths;
        and
         T(q_i) is the joint transformation, R(q_i) for revolute joints Tx(q_i) for 
         prismatic. Type of joint is defined by the @param structure.
        
        Args:
            link_lengths: the lengths of individual links in [m]
            fixed_rotations: fixed joint rotation defined in [rad] [0] * DoF by default
            structure: sequence of joint types, either R or P, [R]*DoF by default
            base_pose: mounting of the robot, identity by default
            gripper_length: length of the gripper measured from the flange
        """
        super().__init__()
        self.link_lengths: np.ndarray = np.asarray(
            [0.5] * 3 if link_lengths is None else link_lengths
        )
        n = len(self.link_lengths)
        self.fixed_rotations: np.ndarray = (
            np.zeros(n) if fixed_rotations is None else fixed_rotations
        )
        self.base_pose = SE2() if base_pose is None else base_pose
        self.structure = ["R"] * n if structure is None else structure
        assert len(self.structure) == len(self.link_lengths)
        self.gripper_length = gripper_length

        # Robot configuration:
        self.q = np.array([np.pi / 8] * n)
        self.gripper_opening = 0.2

    @property
    def dof(self):
        """Return number of degrees of freedom."""
        return len(self.q)

    def flange_pose(self) -> SE2:
        """Return the pose of the flange in the reference frame."""
        # todo HW02: implement fk for the flange
        return SE2()

    def fk_all_links(self) -> list[SE2]:
        """Compute FK for frames that are attached to the joints of the robot."""
        # todo HW02: implement fk, see description of class for information where
        # frames are located
        frames = []
        return frames

    def _gripper_lines(self, flange: SE2):
        """Return tuple of lines (start-end point) that are used to plot gripper
        attached to the flange frame."""
        gripper_opening = self.gripper_opening / 2.0
        return (
            (
                (flange * SE2([0, -gripper_opening])).translation,
                (flange * SE2([0, +gripper_opening])).translation,
            ),
            (
                (flange * SE2([0, -gripper_opening])).translation,
                (flange * SE2([self.gripper_length, -gripper_opening])).translation,
            ),
            (
                (flange * SE2([0, +gripper_opening])).translation,
                (flange * SE2([self.gripper_length, +gripper_opening])).translation,
            ),
        )

    def jacobian(self) -> np.ndarray:
        """Computes Jacobian of the manipulator for the given structure and
        configuration."""
        jac = np.zeros((3, len(self.q)))
        # todo: HW03 implement jacobian computation
        return jac

    def jacobian_finite_difference(self, delta=1e-5) -> np.ndarray:
        jac = np.zeros((3, len(self.q)))
        # todo: HW03 implement jacobian computation numerically
        return jac
