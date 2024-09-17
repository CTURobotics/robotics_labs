#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-10-19
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import unittest
import numpy as np
from robotics_toolbox.utils import circle_circle_intersection, circle_line_intersection


class TestGeometryUtils(unittest.TestCase):
    def test_circ_circ_intersection(self):
        ints = circle_circle_intersection([0, 0], 1, [1, 0], 1)
        self.assertEqual(len(ints), 2)
        (ax, ay), (bx, by) = ints
        if by > ay:
            ay, by = by, ay
        self.assertAlmostEqual(ax, 0.5)
        self.assertAlmostEqual(ay, np.sqrt(1 - 0.5**2))
        self.assertAlmostEqual(bx, 0.5)
        self.assertAlmostEqual(by, -np.sqrt(1 - 0.5**2))

        ints = circle_circle_intersection([0, 0], 1, [2, 0], 1)
        self.assertEqual(len(ints), 2)
        (ax, ay), (bx, by) = ints
        self.assertAlmostEqual(ax, 1.0)
        self.assertAlmostEqual(ay, 0.0)
        self.assertAlmostEqual(bx, 1.0)
        self.assertAlmostEqual(by, 0.0)

        ints = circle_circle_intersection([0, 0], 1, [0, 0], 1)
        self.assertEqual(len(ints), 2)
        a, b = ints
        self.assertAlmostEqual(np.linalg.norm(a), 1.0)
        self.assertAlmostEqual(np.linalg.norm(b), 1.0)

    def test_circ_line_intersection(self):
        ints = circle_line_intersection([0, 0], 1, [-5, 0], [5, 0])
        self.assertEqual(len(ints), 2)
        print(ints)
        (ax, ay), (bx, by) = ints
        if ax > bx:
            ax, bx = bx, ax
            ay, by = by, ay
        self.assertAlmostEqual(ax, -1.0)
        self.assertAlmostEqual(ay, 0.0)
        self.assertAlmostEqual(bx, 1.0)
        self.assertAlmostEqual(by, 0.0)

        ints = circle_line_intersection([0, 0], 2, [0, 5], [0, -5])
        self.assertEqual(len(ints), 2)
        (ax, ay), (bx, by) = ints
        if ay > by:
            ay, by = by, ay
        self.assertAlmostEqual(ax, 0.0)
        self.assertAlmostEqual(ay, -2.0)
        self.assertAlmostEqual(bx, 0.0)
        self.assertAlmostEqual(by, 2.0)


if __name__ == "__main__":
    unittest.main()
