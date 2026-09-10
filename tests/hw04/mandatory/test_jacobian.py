#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-08-29
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import unittest
import numpy as np
from robotics_toolbox.robots import PlanarManipulator
from tests.utils import (
    planar_jacobian_reference,
    sample_planar_manipulator,
    assert_no_forbidden_imports,
)


class TestJacobian(unittest.TestCase):

    def test_analytical_to_reference(self):
        np.random.seed(0)
        for _ in range(100):
            robot = sample_planar_manipulator()
            jac_ref = planar_jacobian_reference(robot)
            jac = robot.jacobian()
            self.assertEqual(jac.shape, (3, robot.dof))
            self.assertEqual(jac_ref.shape, (3, robot.dof))
            self.assertTrue(np.allclose(jac_ref, jac, rtol=1e-4, atol=1e-4))

    def test_finite_difference_to_reference(self):
        np.random.seed(0)
        for _ in range(100):
            robot = sample_planar_manipulator()
            jac_ref = planar_jacobian_reference(robot)
            jac = robot.jacobian_finite_difference()
            self.assertEqual(jac.shape, (3, robot.dof))
            self.assertEqual(jac_ref.shape, (3, robot.dof))
            self.assertTrue(np.allclose(jac_ref, jac, rtol=1e-4, atol=1e-4))

    def test_imported_modules(self):
        """Test that you are not using any external library (scipy, ...) inside your
        implementation, only python standard library and numpy are allowed."""
        assert_no_forbidden_imports(self, PlanarManipulator)


if __name__ == "__main__":
    unittest.main()
