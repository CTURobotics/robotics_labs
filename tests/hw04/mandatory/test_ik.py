#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import inspect
import unittest
import numpy as np

from robotics_toolbox.robots import PlanarManipulator
from tests.utils import sample_planar_manipulator


class TestIK(unittest.TestCase):
    def test_numerical_ik(self):
        np.random.seed(0)
        for i in range(100):
            robot = sample_planar_manipulator(np.random.randint(3, 6))
            robot.q = np.random.uniform(-np.pi, np.pi, size=robot.dof)
            target = robot.flange_pose()

            # random initialization
            robot.q = np.random.uniform(-np.pi, np.pi, size=robot.dof)
            suc = robot.ik_numerical(target, max_iterations=1000, acceptable_err=1e-4)
            self.assertTrue(suc)
            diff = target.inverse() * robot.flange_pose()
            self.assertLess(np.abs(diff.translation[0]), 1e-4)
            self.assertLess(np.abs(diff.translation[1]), 1e-4)
            self.assertLess(np.abs(diff.rotation.angle), 1e-4)

    def test_analytical_ik_rrr(self):
        np.random.seed(0)
        for i in range(100):
            robot = sample_planar_manipulator(3)
            robot.structure = "RRR"
            robot.q = np.random.uniform(-np.pi, np.pi, size=robot.dof)
            target = robot.flange_pose()

            # random q
            robot.q = np.random.uniform(-np.pi, np.pi, size=robot.dof)

            solutions = robot.ik_analytical(target)
            self.assertEqual(len(solutions), 2)
            for sol in solutions:
                self.assertTrue(np.all(np.abs(sol) <= np.pi))
                robot.q = sol
                diff = target.inverse() * robot.flange_pose()
                self.assertLess(np.abs(diff.translation[0]), 1e-4)
                self.assertLess(np.abs(diff.translation[1]), 1e-4)
                self.assertLess(np.abs(diff.rotation.angle), 1e-4)

    def test_imported_modules(self):
        """Test that you are not using pinocchio inside your implementation."""
        with open(inspect.getfile(PlanarManipulator)) as f:
            self.assertTrue("pinocchio" not in f.read())
        with open(inspect.getfile(PlanarManipulator)) as f:
            self.assertTrue("cv2" not in f.read())
        with open(inspect.getfile(PlanarManipulator)) as f:
            self.assertTrue("scipy" not in f.read())
