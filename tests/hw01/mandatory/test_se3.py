#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-4
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import unittest
import numpy as np
from numpy.testing import assert_allclose

from robotics_toolbox.core import SE3, SO3
from tests.utils import exp3, hom3, assert_no_forbidden_imports


class TestSE3(unittest.TestCase):
    """SE3 class depends on SO3, implement SO3 first."""

    def test_initialization(self):
        """Constructor implemented, this test pass by default."""
        self.assertEqual(SE3(), SE3(np.zeros(3), SO3()))

    def test_act(self):
        np.random.seed(0)
        for _ in range(100):
            v = np.random.rand(3)
            t = np.random.rand(3)
            a = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            v_ = SE3(t, SO3(exp3(a))).act(v)
            ref_v_ = hom3(exp3(a), t) @ np.append(v, 1)
            self.assertTrue(np.allclose(v_, ref_v_[:3]))

    def test_inverse(self):
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            t = np.random.uniform(-10, 10, size=3)
            r = SO3(exp3(a))
            pose = SE3(t, r)
            exp = np.linalg.inv(pose.homogeneous())
            assert_allclose(pose.inverse().homogeneous(), exp)

    def test_composition(self):
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            t = np.random.uniform(-10, 10, size=3)
            a_ = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            t_ = np.random.uniform(-10, 10, size=3)

            ta = SE3(t, SO3(exp3(a)))
            tb = SE3(t_, SO3(exp3(a_)))
            tc = ta * tb

            m = hom3(exp3(a), t) @ hom3(exp3(a_), t_)

            self.assertTrue(np.allclose(m[:3, :3], tc.rotation.rot))
            self.assertTrue(np.allclose(m[:3, 3], tc.translation))

    def test_only_rotation_and_translation_variables(self):
        t = SE3()
        all_vars = list(vars(t).keys())
        self.assertEqual(len(all_vars), 2)
        self.assertTrue("translation" in all_vars)
        self.assertTrue("rotation" in all_vars)

    def test_imported_modules(self):
        """Test that you are not using any external library (scipy, ...) inside your
        implementation, only python standard library and numpy are allowed."""
        assert_no_forbidden_imports(self, SE3)


if __name__ == "__main__":
    unittest.main()
