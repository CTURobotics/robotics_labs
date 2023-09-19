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
from pinocchio import OP_FRAME

from robotics_toolbox.core import SE2, SO2
from robotics_toolbox.robots import PlanarManipulator


class TestFKPlanar(unittest.TestCase):
    @staticmethod
    def _se2_to_pin_se3(pose: SE2) -> pin.SE3:
        return pin.SE3(
            pin.exp(np.asarray([0, 0, pose.rotation.angle])),
            np.pad(pose.translation, (0, 1)),
        )

    def _assert_se2_equal_pin_se3(self, a: SE2, b: pin.SE3):
        m = b.homogeneous
        self.assertTrue(np.allclose(m[:2, :2], a.rotation.rot))
        self.assertTrue(np.allclose(m[:2, 3], a.translation))

    def _convert_planar_manipualtor_to_pin(self, robot: PlanarManipulator) -> pin.Model:
        model: pin.Model = pin.Model()
        jid = 0
        pose = robot.base_pose
        for li, qi, jtype in zip(robot.link_lengths, robot.q, robot.structure):
            if jtype == "R":
                jid = model.addJoint(
                    jid, pin.JointModelRZ(), self._se2_to_pin_se3(pose), f"j{jid}"
                )
                pose = SE2(translation=[li, 0])
            elif jtype == "P":
                pose *= SE2(rotation=SO2(li))
                jid = model.addJoint(
                    jid, pin.JointModelPX(), self._se2_to_pin_se3(pose), f"j{jid}"
                )
                pose = SE2()

        model.addFrame(
            pin.Frame(f"flange", jid, 0, self._se2_to_pin_se3(pose), OP_FRAME)
        )
        return model

    def test_fk_all_links(self):
        np.random.seed(0)
        n = 3
        robot = PlanarManipulator(
            link_lengths=np.random.uniform(0.1, 0.3, size=n),
            base_pose=SE2(
                translation=np.random.uniform(-0.5, 0.5, size=2),
                rotation=SO2(np.random.uniform(-np.pi, np.pi)),
            ),
            structure=np.random.choice(["R", "P"], size=n),
        )

        for _ in range(10):
            robot.q = np.random.uniform(-np.pi, np.pi, size=robot.dof)

            model = self._convert_planar_manipualtor_to_pin(robot)
            data: pin.Data = model.createData()
            pin.forwardKinematics(model, data, robot.q)
            pin.updateFramePlacements(model, data)

            frames = robot.fk_all_links()
            self.assertEqual(len(frames), robot.dof + 1)
            self.assertEqual(frames[0], robot.base_pose)

            for fref, f, qi, jt, li in zip(
                data.oMi[1:], frames[1:], robot.q, robot.structure, robot.link_lengths
            ):
                d = SE2([-li, 0]) if jt == "R" else SE2()
                self._assert_se2_equal_pin_se3(f * d, fref)
            self._assert_se2_equal_pin_se3(frames[-1], data.oMf[1])
            self._assert_se2_equal_pin_se3(robot.flange_pose(), data.oMf[1])

    def test_imported_modules(self):
        """Test that you are not using pinocchio inside your implementation."""
        with open(inspect.getfile(PlanarManipulator)) as f:
            self.assertTrue("pinocchio" not in f.read())


if __name__ == "__main__":
    unittest.main()
