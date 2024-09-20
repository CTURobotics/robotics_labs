#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-18
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import inspect
import unittest

import numpy as np
import pinocchio as pin

from robotics_toolbox.core import SE2
from robotics_toolbox.robots import PlanarManipulator
from tests.utils import (
    planar_manipulator_to_pin,
    assert_se2_equals_pin_se3,
    sample_planar_manipulator,
)


class TestFKPlanar(unittest.TestCase):

    def test_flange_pose(self):
        np.random.seed(0)
        for _ in range(100):
            robot = sample_planar_manipulator()
            robot.q = np.random.uniform(-np.pi, np.pi, size=robot.dof)
            model = planar_manipulator_to_pin(robot)
            data: pin.Data = model.createData()
            pin.forwardKinematics(model, data, robot.q)
            pin.updateFramePlacements(model, data)
            assert_se2_equals_pin_se3(self, robot.flange_pose(), data.oMf[1])

    def test_fk_all_links(self):
        np.random.seed(0)
        for _ in range(100):
            robot = sample_planar_manipulator()
            robot.q = np.random.uniform(-np.pi, np.pi, size=robot.dof)

            model = planar_manipulator_to_pin(robot)
            data: pin.Data = model.createData()
            pin.forwardKinematics(model, data, robot.q)
            pin.updateFramePlacements(model, data)

            frames = robot.fk_all_links()
            self.assertEqual(len(frames), robot.dof + 1)
            self.assertEqual(frames[0], robot.base_pose)

            for fref, f, qi, jt, li in zip(
                data.oMi[1:],
                frames[1:],
                robot.q,
                robot.structure,
                robot.link_parameters,
            ):
                d = SE2([-li, 0]) if jt == "R" else SE2()
                assert_se2_equals_pin_se3(self, f * d, fref)
            assert_se2_equals_pin_se3(self, frames[-1], data.oMf[1])

    def test_imported_modules(self):
        """Test that you are not using pinocchio inside your implementation."""
        with open(inspect.getfile(PlanarManipulator)) as f:
            self.assertTrue("pinocchio" not in f.read())
        with open(inspect.getfile(PlanarManipulator)) as f:
            self.assertTrue("cv2" not in f.read())
        with open(inspect.getfile(PlanarManipulator)) as f:
            self.assertTrue("scipy" not in f.read())


if __name__ == "__main__":
    unittest.main()
