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

from robotics_toolbox.core import SE3, SO3


class TestSE3(unittest.TestCase):
    def test_initialization(self):
        self.assertEqual(SE3(), SE3(np.zeros(3), SO3()))

    def test_only_rotation_and_translation_variables(self):
        t = SE3()
        all_vars = list(vars(t).keys())
        self.assertEqual(len(all_vars), 2)
        self.assertTrue("translation" in all_vars)
        self.assertTrue("rotation" in all_vars)

    def test_act(self):
        np.random.seed(0)
        for _ in range(100):
            v = np.random.rand(3)
            t = np.random.rand(3)
            a = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            v_ = SE3(t, SO3(pin.exp(a))).act(v)
            ref_t = pin.SE3(pin.exp(a), t)
            ref_v_ = ref_t.act(v)
            self.assertTrue(np.allclose(v_, ref_v_))

    def test_imported_modules(self):
        """Test that you are not using pinocchio inside your implementation."""
        with open(inspect.getfile(SE3)) as f:
            self.assertTrue("pinocchio" not in f.read())

    def test_inverse(self):
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            t = np.random.uniform(-10, 10, size=3)
            r = SO3(pin.exp(a))
            self.assertEqual(SE3(t, r) * SE3(t, r).inverse(), SE3())
            self.assertEqual(SE3(t, r).inverse() * SE3(t, r), SE3())

    def test_composition(self):
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            t = np.random.uniform(-10, 10, size=3)
            a_ = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            t_ = np.random.uniform(-10, 10, size=3)

            ta = SE3(t, SO3(pin.exp(a)))
            tb = SE3(t_, SO3(pin.exp(a_)))
            tc = ta * tb

            pin_ta = pin.SE3(pin.exp(a), t)
            pin_tb = pin.SE3(pin.exp(a_), t_)
            pin_c: pin.SE3 = pin_ta * pin_tb
            m = pin_c.homogeneous

            self.assertTrue(np.allclose(m[:3, :3], tc.rotation.rot))
            self.assertTrue(np.allclose(m[:3, 3], tc.translation))


if __name__ == "__main__":
    unittest.main()
