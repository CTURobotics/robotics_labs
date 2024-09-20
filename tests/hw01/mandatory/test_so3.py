#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-4
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import unittest
import numpy as np
from numpy.testing import assert_allclose
import inspect
import pinocchio as pin

from robotics_toolbox.core import SO3


class TestSO3(unittest.TestCase):
    @staticmethod
    def so3_from_exp(v):
        return SO3(pin.exp(v))

    def test_so3_initialization(self):
        """Works by default, no implementation needed."""
        self.assertEqual(SO3().rot.shape, (3, 3))
        assert_allclose(SO3().rot, np.eye(3))

    def test_exp(self):
        """Require to implement exponential map."""
        np.random.seed(0)
        for _ in range(100):
            v = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            self.assertEqual(SO3.exp(v), self.so3_from_exp(v))

    def test_log(self):
        """Require to implement logarithm mapping from SO3, s.t. exp(log(T)) == T."""

        def compute_so3_through_log(v):
            return self.so3_from_exp(self.so3_from_exp(v).log())

        np.random.seed(0)
        for _ in range(100):
            v = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            self.assertEqual(self.so3_from_exp(v), compute_so3_through_log(v))
        v = np.array([0, 0, 0])
        self.assertEqual(self.so3_from_exp(v), compute_so3_through_log(v))
        v = np.array([0, 0, np.pi])
        self.assertEqual(self.so3_from_exp(v), compute_so3_through_log(v))
        v = np.array([np.pi, 0, 0])
        self.assertEqual(self.so3_from_exp(v), compute_so3_through_log(v))

    def test_inverse(self):
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            t = self.so3_from_exp(a)
            assert_allclose(t.inverse().rot, np.linalg.inv(t.rot))

    def test_composition(self):
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            b = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            c = self.so3_from_exp(a) * self.so3_from_exp(b)
            self.assertEqual(c, SO3(pin.exp(a) @ pin.exp(b)))

    def test_act(self):
        """Passes by default"""
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            v = np.random.rand(3)
            v_ = self.so3_from_exp(a).act(v)
            self.assertTrue(np.allclose(v_, pin.exp(a) @ v))

    def test_only_rot_variable(self):
        """Test that SO3 has only rot variable. Pass by default."""
        t = SO3()
        all_vars = list(vars(t).keys())
        self.assertEqual(len(all_vars), 1)
        self.assertEqual(all_vars[0], "rot")

    def test_imported_modules(self):
        """Test that you are not using pinocchio inside your implementation.
        Pass by default."""
        with open(inspect.getfile(SO3)) as f:
            self.assertTrue("pinocchio" not in f.read())
        with open(inspect.getfile(SO3)) as f:
            self.assertTrue("scipy" not in f.read())
        with open(inspect.getfile(SO3)) as f:
            self.assertTrue("cv2" not in f.read())


if __name__ == "__main__":
    unittest.main()
