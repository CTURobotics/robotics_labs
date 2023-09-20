#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

import unittest
import numpy as np

from tests.utils import sample_planar_manipulator


class TestIKPRR(unittest.TestCase):
    def test_analytical_ik_prr(self):
        np.random.seed(0)
        for i in range(100):
            robot = sample_planar_manipulator(3)
            robot.structure = "PRR"
            robot.q = np.random.uniform(-np.pi, np.pi, size=robot.dof)
            robot.q[0] = np.random.uniform(-10, 10)
            target = robot.flange_pose()

            # random q
            robot.q = np.random.uniform(-np.pi, np.pi, size=robot.dof)
            solutions = robot.ik_analytical(target)
            self.assertEqual(len(solutions), 2)
            for sol in solutions:
                self.assertTrue(np.all(np.abs(sol[1:]) <= np.pi))
                robot.q = sol
                diff = target.inverse() * robot.flange_pose()
                self.assertLess(np.abs(diff.translation[0]), 1e-4)
                self.assertLess(np.abs(diff.translation[1]), 1e-4)
                self.assertLess(np.abs(diff.rotation.angle), 1e-4)
