#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-4
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import unittest
import numpy as np
import inspect
import pinocchio as pin

from robotics_toolbox.core import SO2


class TestSO2(unittest.TestCase):
    def test_so2_initialization(self):
        self.assertEqual(SO2().rot.shape, (2, 2))
        self.assertAlmostEqual(SO2(np.pi / 7).angle, np.pi / 7)
        self.assertAlmostEqual(SO2(np.pi).angle, np.pi)

    def test_only_rot_variable(self):
        t = SO2()
        all_vars = list(vars(t).keys())
        self.assertEqual(len(all_vars), 1)
        self.assertEqual(all_vars[0], "rot")

    def test_reference_rotations(self):
        for a in np.linspace(-2 * np.pi, 2 * np.pi, num=1000):
            ref_rot = pin.exp(np.array([0, 0, a]))[:2, :2]
            t = SO2(a)
            self.assertTrue(np.allclose(ref_rot, t.rot))

    def test_act(self):
        np.random.seed(0)
        for _ in range(10):
            v = np.random.rand(2)
            a = np.random.rand(1)[0]
            t = SO2(a)
            v_ = t.act(v)
            ref_rot = pin.exp(np.array([0, 0, a]))[:2, :2]
            self.assertTrue(np.allclose(v_, ref_rot @ v))

    def test_act_trivial(self):
        np.random.seed(0)
        v = np.random.rand(2)
        self.assertTrue(np.allclose(SO2(angle=np.pi).act(v), -v))
        self.assertTrue(np.allclose(SO2(angle=np.pi / 2).act(v), [-v[1], v[0]]))
        self.assertTrue(np.allclose(SO2(angle=-np.pi / 2).act(v), [v[1], -v[0]]))

    def test_imported_modules(self):
        """Test that you are not using pinocchio inside your implementation."""
        with open(inspect.getfile(SO2)) as f:
            self.assertTrue("pinocchio" not in f.read())

    def test_inverse(self):
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi)
            self.assertEqual(SO2(a) * SO2(a).inverse(), SO2())

    def test_composition(self):
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi)
            b = np.random.uniform(-2 * np.pi, 2 * np.pi)
            self.assertEqual(SO2(a) * SO2(b), SO2(a + b))


if __name__ == "__main__":
    unittest.main()
