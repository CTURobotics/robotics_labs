#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-22
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

import unittest
import numpy as np

from robotics_toolbox.core import SE2, SO3, SE3
from robotics_toolbox.utils import interpolate, distance_between_configurations


class TestConfigurationUtils(unittest.TestCase):
    def test_interpolation_array(self):
        a = np.random.uniform(low=-np.pi, high=np.pi, size=5)
        b = np.random.uniform(low=-np.pi, high=np.pi, size=5)
        d = 0.31
        c = interpolate(a, b, d)
        self.assertAlmostEqual(distance_between_configurations(a, c), d)

    def test_interpolation_se2(self):
        a = SE2(
            translation=np.random.uniform(low=-10, high=10, size=2),
            rotation=np.random.uniform(low=-np.pi, high=np.pi),
        )
        b = SE2(
            translation=np.random.uniform(low=-10, high=10, size=2),
            rotation=np.random.uniform(low=-np.pi, high=np.pi),
        )
        d = 0.31
        c = interpolate(a, b, d)
        self.assertAlmostEqual(distance_between_configurations(a, c), d)

    def test_interpolation_se3(self):
        a = SE3(
            translation=np.random.uniform(low=-10, high=10, size=3),
            rotation=SO3.exp(np.random.uniform(low=-np.pi, high=np.pi, size=3)),
        )
        b = SE3(
            translation=np.random.uniform(low=-10, high=10, size=3),
            rotation=SO3.exp(np.random.uniform(low=-np.pi, high=np.pi, size=3)),
        )
        d = 0.31
        c = interpolate(a, b, d)
        self.assertAlmostEqual(distance_between_configurations(a, c), d)


if __name__ == "__main__":
    unittest.main()
