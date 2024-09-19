#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-10-4
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from __future__ import annotations

import unittest
from copy import deepcopy

import numpy as np
from numpy.random import uniform
from robotics_toolbox.core import SO2, SO3, SE3, SE2


class TestImmutability(unittest.TestCase):
    @staticmethod
    def sample(n=1) -> list[SO2 | SO3 | SE3 | SE2]:
        """Sample n instances of one of the core space."""
        cls = np.random.choice([SO2, SO3, SE3, SE2])
        if cls == SO2:
            return [cls(uniform(-np.pi, np.pi)) for _ in range(n)]
        elif cls == SO3:
            return [cls.exp(uniform(-np.pi, np.pi, 3)) for _ in range(n)]
        elif cls == SE2:
            return [cls(uniform(-10, 10, 2), uniform(-np.pi, np.pi)) for _ in range(n)]
        elif cls == SE3:
            return [
                cls(uniform(-10, 10, 3), SO3.exp(uniform(-np.pi, np.pi, 3)))
                for _ in range(n)
            ]

    def test_immutable(self):
        """Multiplication, inverse and act should not modify instances."""
        np.random.seed(0)
        for _ in range(100):
            a, b = self.sample(n=2)
            aa, bb = deepcopy(a), deepcopy(b)
            _ = a * b
            self.assertEqual(a, aa)
            self.assertEqual(b, bb)
            _ = a.inverse()
            self.assertEqual(a, aa)
            _ = a.act(np.random.uniform(-10, 10, 2 if isinstance(a, (SO2, SE2)) else 3))
            self.assertEqual(a, aa)


if __name__ == "__main__":
    unittest.main()
