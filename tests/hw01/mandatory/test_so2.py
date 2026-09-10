#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-4
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import unittest
import numpy as np

from robotics_toolbox.core import SO2
from tests.utils import rot2, assert_no_forbidden_imports


class TestSO2(unittest.TestCase):
    @staticmethod
    def _norm_angle(angle: float) -> float:
        """Normalize angle to the interval [-pi, pi]."""
        return np.arctan2(np.sin(angle), np.cos(angle))

    def asert_angles_equal(self, a: float, b: float):
        """Check if two angles are equal."""
        self.assertAlmostEqual(self._norm_angle(a), self._norm_angle(b))

    @staticmethod
    def _so2_from_angle(angle: float) -> SO2:
        """Create SO2 from angle."""
        r = SO2()
        r.rot = rot2(angle)
        return r

    def test_so2_initialization(self):
        """This test requires implementation of constructor of SO2."""
        self.assertEqual(SO2().rot.shape, (2, 2))
        self.assertTrue(np.allclose(SO2().rot, np.eye(2)))
        self.assertTrue(np.allclose(SO2(np.pi).rot, np.array([[-1, 0], [0, -1]])))
        self.assertTrue(np.allclose(SO2(np.pi / 2).rot, np.array([[0, -1], [1, 0]])))
        self.assertTrue(np.allclose(SO2(-np.pi / 2).rot, np.array([[0, 1], [-1, 0]])))

    def test_reference_rotations(self):
        """This test requires implementation of constructor of SO2."""
        for a in np.linspace(-2 * np.pi, 2 * np.pi, num=1000):
            ref_rot = rot2(a)
            t = SO2(a)
            self.assertTrue(np.allclose(ref_rot, t.rot))

    def test_angle(self):
        """This test requires implementation of angle property of SO2."""
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi)
            r = self._so2_from_angle(a)
            self.asert_angles_equal(r.angle, a)

    def test_act(self):
        """This test requires implementation of act method of SO2."""
        np.random.seed(0)
        for _ in range(10):
            v = np.random.rand(2)
            a = np.random.rand(1)[0]
            t = self._so2_from_angle(a)
            v_ = t.act(v)
            ref_rot = rot2(a)
            self.assertTrue(np.allclose(v_, ref_rot @ v))

    def test_act_trivial(self):
        np.random.seed(0)
        v = np.random.rand(2)
        self.assertTrue(np.allclose(self._so2_from_angle(np.pi).act(v), -v))
        self.assertTrue(
            np.allclose(self._so2_from_angle(np.pi / 2).act(v), [-v[1], v[0]])
        )
        self.assertTrue(
            np.allclose(self._so2_from_angle(-np.pi / 2).act(v), [v[1], -v[0]])
        )

    def test_inverse(self):
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi)
            r = self._so2_from_angle(a)
            self.assertTrue(np.allclose(r.rot @ r.inverse().rot, np.eye(2)))

    def test_composition(self):
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi)
            b = np.random.uniform(-2 * np.pi, 2 * np.pi)
            c = self._so2_from_angle(a) * self._so2_from_angle(b)
            self.assertEqual(c, self._so2_from_angle(a + b))

    def test_only_rot_variable(self):
        """This test checks that only rot variable is stored in the class. Not needed
        to implement anything else for it to pass."""
        t = SO2()
        all_vars = list(vars(t).keys())
        self.assertEqual(len(all_vars), 1)
        self.assertEqual(all_vars[0], "rot")

    def test_imported_modules(self):
        """Test that you are not using any external library (scipy, ...) inside your
        implementation, only python standard library and numpy are allowed."""
        assert_no_forbidden_imports(self, SO2)


if __name__ == "__main__":
    unittest.main()
