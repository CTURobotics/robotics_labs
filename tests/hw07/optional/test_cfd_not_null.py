#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-12-7
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import unittest
import numpy as np
from robotics_toolbox.robots.planar_manipulator_dynamics import (
    PlanarManipulatorDynamics,
)


class TestDynamicsNotZero(unittest.TestCase):
    def test_constraint_forward_dynamics_not_zero(self):
        masses = np.random.uniform(0.1, 1, size=2)
        link_parameters = [0, np.random.uniform(0.1, 1)]
        robot = PlanarManipulatorDynamics(
            link_parameters=link_parameters, masses=masses, structure=["P", "R"]
        )
        robot.q = np.random.uniform(-np.pi, np.pi, size=robot.dof)
        q = robot.q
        dq = np.zeros(robot.dof)
        tau = np.zeros(robot.dof)
        ddq = robot.constrained_forward_dynamics(q, dq, tau, damping=0)
        self.assertGreater(np.linalg.norm(ddq), 1e-3)


if __name__ == "__main__":
    unittest.main()
