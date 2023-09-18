#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-08-29
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import unittest
import numpy as np
from robotics_toolbox.robots import PlanarManipulator


class Jacobian(unittest.TestCase):
    def test_analytical_to_finite_difference(self):
        np.random.seed(0)
        for _ in range(100):
            n = np.random.randint(1, 5)
            robot = PlanarManipulator(
                link_lengths=np.random.uniform(0.1, 0.9, size=n),
                structure=np.random.choice(["R", "P"], size=n),
            )
            jac_fd = robot.jacobian_finite_difference()
            jac = robot.jacobian()
            self.assertEqual(jac.shape, (3, n))
            self.assertEqual(jac_fd.shape, (3, n))
            self.assertTrue(np.allclose(jac_fd, jac, rtol=1e-4, atol=1e-4))


if __name__ == "__main__":
    unittest.main()
