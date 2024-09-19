#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-4
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import unittest
import numpy as np
import pinocchio as pin

from robotics_toolbox.core import SO3


class TestSO3Opt(unittest.TestCase):
    def test_rx(self):
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi)
            self.assertEqual(SO3.rx(a), SO3(pin.exp(np.array([a, 0, 0]))))

    def test_ry(self):
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi)
            self.assertEqual(SO3.ry(a), SO3(pin.exp(np.array([0, a, 0]))))

    def test_rz(self):
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi)
            self.assertEqual(SO3.rz(a), SO3(pin.exp(np.array([0, 0, a]))))

    def test_from_q(self):
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            r = SO3(pin.exp(a))
            q = pin.Quaternion(r.rot)
            self.assertEqual(SO3.from_quaternion(q.coeffs()), r)

    def test_to_q(self):
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            r = SO3(pin.exp(a))
            q = r.to_quaternion()
            self.assertEqual(r, SO3(pin.Quaternion(q).toRotationMatrix()))

    def test_from_ea(self):
        np.random.seed(0)
        axes = ["x", "y", "z"]
        ind = {"x": 0, "y": 1, "z": 2}
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            s1 = np.random.choice(axes)
            s2 = np.random.choice([s for s in axes if s != s1])
            s3 = np.random.choice([s for s in axes if s != s2])
            r = SO3.from_euler_angles(a, s1 + s2 + s3)
            rot = np.eye(3)
            for ax, av in zip([ind[s1], ind[s2], ind[s3]], a):
                v = np.zeros(3)
                v[ax] = av
                rot = rot @ pin.exp(v)
            self.assertEqual(r, SO3(rot))

    def test_from_angle_axis(self):
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            angle = np.linalg.norm(a)
            axis = a / angle
            self.assertEqual(SO3.from_angle_axis(angle, axis), SO3(pin.exp(a)))

    def test_to_angle_axis(self):
        np.random.seed(0)
        for _ in range(100):
            a = np.random.uniform(-2 * np.pi, 2 * np.pi, size=3)
            angle, axis = SO3(pin.exp(a)).to_angle_axis()
            self.assertEqual(SO3(pin.exp(a)), SO3(pin.exp(angle * axis)))


if __name__ == "__main__":
    unittest.main()
