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

from robotics_toolbox.core import SE2, SO2


class TestSE2(unittest.TestCase):
    """SE2 testing depends on SO2 implementation, there it is needed to implement SO2
    first."""

    def test_initialization(self):
        """Test constructor. No implementation needed."""
        self.assertEqual(SE2(), SE2(np.zeros(2), SO2(angle=0.0)))
        self.assertEqual(SE2(), SE2(np.zeros(2), 0.0))

    def test_act(self):
        """Required to implement act to pass."""
        np.random.seed(0)
        for _ in range(100):
            v = np.random.rand(2)
            t = np.random.rand(2)
            a = np.random.rand(1)[0]
            v_ = SE2(t, SO2(a)).act(v)
            ref_t = pin.SE3(pin.exp(np.array([0, 0, a])), np.append(t, 0))
            ref_v_ = ref_t.act(np.append(v, 0))
            self.assertTrue(np.allclose(v_, ref_v_[:2]))

    def test_inverse(self):
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi)
            t = np.random.uniform(-10, 10, size=2)
            pose = SE2(t, a)
            m = pose.inverse().homogeneous()
            self.assertTrue(np.allclose(np.linalg.inv(pose.homogeneous()), m))

    def test_composition(self):
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi)
            t = np.random.uniform(-10, 10, size=2)
            a_ = np.random.uniform(-2 * np.pi, 2 * np.pi)
            t_ = np.random.uniform(-10, 10, size=2)
            ta = SE2(t, SO2(a))
            tb = SE2(t_, SO2(a_))
            tc = ta * tb

            pin_ta = pin.SE3(pin.exp(np.array([0, 0, a])), np.append(t, 0))
            pin_tb = pin.SE3(pin.exp(np.array([0, 0, a_])), np.append(t_, 0))
            pin_c: pin.SE3 = pin_ta * pin_tb
            m = pin_c.homogeneous

            self.assertTrue(np.allclose(m[:2, :2], tc.rotation.rot))
            self.assertTrue(np.allclose(m[:2, 3], tc.translation))

    def test_imported_modules(self):
        """Test that you are not using pinocchio inside your implementation. Passing by
        default."""
        with open(inspect.getfile(SE2)) as f:
            self.assertTrue("pinocchio" not in f.read())
        with open(inspect.getfile(SE2)) as f:
            self.assertTrue("scipy" not in f.read())
        with open(inspect.getfile(SE2)) as f:
            self.assertTrue("cv2" not in f.read())

    def test_only_rotation_and_translation_variables(self):
        """Test that SE2 has only rotation and translation variables. Passing by
        default."""
        t = SE2()
        all_vars = list(vars(t).keys())
        self.assertEqual(len(all_vars), 2)
        self.assertTrue("translation" in all_vars)
        self.assertTrue("rotation" in all_vars)


if __name__ == "__main__":
    unittest.main()
