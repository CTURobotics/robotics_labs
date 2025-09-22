#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-08-29
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import inspect
import unittest
import numpy as np
import pinocchio as pin
from robotics_toolbox.robots import PlanarManipulator
from tests.utils import planar_manipulator_to_pin, sample_planar_manipulator


class TestJacobian(unittest.TestCase):

    def _pin_jacobian(self, robot):
        model = planar_manipulator_to_pin(robot)
        data: pin.Data = model.createData()
        pin.forwardKinematics(model, data, robot.q)
        pin.updateFramePlacements(model, data)
        jac = pin.computeFrameJacobian(
            model, data, robot.q, 1, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        if robot.dof == 1:
            jac = jac[:, np.newaxis]
        jac = jac[[0, 1, 5], :]
        return jac

    def test_analytical_to_pin(self):
        np.random.seed(0)
        for _ in range(100):
            robot = sample_planar_manipulator()
            jac_pin = self._pin_jacobian(robot)
            jac = robot.jacobian()
            self.assertEqual(jac.shape, (3, robot.dof))
            self.assertEqual(jac_pin.shape, (3, robot.dof))
            self.assertTrue(np.allclose(jac_pin, jac, rtol=1e-4, atol=1e-4))

    def test_finite_difference_to_pin(self):
        np.random.seed(0)
        for _ in range(100):
            robot = sample_planar_manipulator()
            jac_pin = self._pin_jacobian(robot)
            jac = robot.jacobian_finite_difference()
            self.assertEqual(jac.shape, (3, robot.dof))
            self.assertEqual(jac_pin.shape, (3, robot.dof))
            self.assertTrue(np.allclose(jac_pin, jac, rtol=1e-4, atol=1e-4))

    def test_imported_modules(self):
        """Test that you are not using pinocchio inside your implementation."""
        with open(inspect.getfile(PlanarManipulator)) as f:
            self.assertTrue("pinocchio" not in f.read())
        with open(inspect.getfile(PlanarManipulator)) as f:
            self.assertTrue("cv2" not in f.read())
        with open(inspect.getfile(PlanarManipulator)) as f:
            self.assertTrue("scipy" not in f.read())


if __name__ == "__main__":
    unittest.main()
