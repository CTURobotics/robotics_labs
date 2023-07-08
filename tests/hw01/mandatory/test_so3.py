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

from robotics_toolbox.core import SO3


class TestSO3(unittest.TestCase):
    def test_so2_initialization(self):
        self.assertEqual(SO3().rot.shape, (3, 3))

    def test_only_rot_variable(self):
        t = SO3()
        all_vars = list(vars(t).keys())
        self.assertEqual(len(all_vars), 1)
        self.assertEqual(all_vars[0], "rot")

    def test_exp(self):
        np.random.seed(0)
        for _ in range(100):
            v = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            self.assertEqual(SO3.exp(v), SO3(pin.exp(v)))

    def test_log(self):
        np.random.seed(0)
        for _ in range(100):
            v = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            self.assertEqual(SO3.exp(v), SO3.exp(SO3.exp(v).log()))
        v = [0, 0, 0]
        self.assertEqual(SO3.exp(v), SO3.exp(SO3.exp(v).log()))
        v = [0, 0, np.pi]
        self.assertEqual(SO3.exp(v), SO3.exp(SO3.exp(v).log()))
        v = [np.pi, 0, 0]
        self.assertEqual(SO3.exp(v), SO3.exp(SO3.exp(v).log()))

    def test_act(self):
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            v = np.random.rand(3)
            v_ = SO3.exp(a).act(v)
            self.assertTrue(np.allclose(v_, pin.exp(a) @ v))

    def test_imported_modules(self):
        """Test that you are not using pinocchio inside your implementation."""
        with open(inspect.getfile(SO3)) as f:
            self.assertTrue("pinocchio" not in f.read())

    def test_inverse(self):
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            t = SO3.exp(a)
            self.assertEqual(t * t.inverse(), SO3())

    def test_composition(self):
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            b = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            self.assertEqual(SO3.exp(a) * SO3.exp(b), SO3(pin.exp(a) @ pin.exp(b)))


if __name__ == "__main__":
    unittest.main()
