#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-12-2
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import inspect
import unittest
import numpy as np
import pinocchio as pin
from robotics_toolbox.core import SO2, SE2
from robotics_toolbox.robots.planar_manipulator_dynamics import (
    PlanarManipulatorDynamics,
)
from tests.utils import se2_to_pin_se3


class TestPlanarManipulatorDynamics(unittest.TestCase):
    def _sample_manipulator(self):
        masses = np.random.uniform(0.1, 1, size=2)
        link_parameters = [0, np.random.uniform(0.1, 1)]
        robot = PlanarManipulatorDynamics(
            link_parameters=link_parameters, masses=masses, structure=["P", "R"]
        )
        robot.q = np.random.uniform(-np.pi, np.pi, size=robot.dof)

        model = self.planar_manipulator_to_pin(robot)
        data = model.createData()
        return robot, model, data

    def test_mass_matrix(self):
        np.random.seed(0)
        for _ in range(100):
            robot, model, data = self._sample_manipulator()
            q = robot.q
            m = robot.mass_matrix(q)
            m_expected = pin.crba(model, data, q)
            np.testing.assert_allclose(m, m_expected)

    def test_h(self):
        np.random.seed(0)
        for _ in range(100):
            robot, model, data = self._sample_manipulator()
            q = robot.q
            dq = np.random.uniform(-np.pi, np.pi, size=robot.dof)
            h = robot.h(q, dq)
            h_expected = pin.rnea(model, data, q, dq, np.zeros(robot.dof))
            np.testing.assert_allclose(h, h_expected)

    def test_unconstrained_fd(self):
        np.random.seed(0)
        for _ in range(100):
            robot, model, data = self._sample_manipulator()
            q = robot.q
            dq = np.random.uniform(-np.pi, np.pi, size=robot.dof)
            tau = np.random.uniform(-np.pi, np.pi, size=robot.dof)
            damping = np.random.uniform(0, 1)

            ddq = robot.forward_dynamics(q=q, dq=dq, tau=tau, damping=damping)
            ddq_expected = pin.aba(model, data, q, dq, tau - damping * dq)
            np.testing.assert_allclose(ddq, ddq_expected)

    def test_unconstrained_id(self):
        np.random.seed(0)
        for _ in range(100):
            robot, model, data = self._sample_manipulator()
            q = robot.q
            dq = np.random.uniform(-np.pi, np.pi, size=robot.dof)
            ddq = np.random.uniform(-np.pi, np.pi, size=robot.dof)
            damping = np.random.uniform(0, 1)
            tau = robot.inverse_dynamics(q=q, dq=dq, ddq=ddq, damping=damping)
            tau_expected = pin.rnea(model, data, q, dq, ddq) + damping * dq
            np.testing.assert_allclose(tau, tau_expected)

    def test_constrained_fd(self):
        np.random.seed(0)
        for _ in range(10):
            robot, model, data = self._sample_manipulator()
            q = robot.q
            dq = np.zeros(2)
            tau = np.random.uniform(-np.pi, np.pi, size=robot.dof)
            damping = np.random.uniform(0, 1)

            x = robot.flange_pose().translation.copy()
            angle = np.deg2rad(45)
            dt = 0.0001
            for t in range(1000):
                ddq = robot.constrained_forward_dynamics(q, dq, tau, damping=damping)

                dq += ddq * dt
                q += dq * dt

                robot.q = q
                pt = robot.flange_pose().translation
                point_line_d = np.abs(
                    np.cos(angle) * (x[1] - pt[1]) - np.sin(angle) * (x[0] - pt[0])
                )
                self.assertLess(point_line_d, 1e-3)

    @staticmethod
    def planar_manipulator_to_pin(robot: PlanarManipulatorDynamics) -> pin.Model:
        model: pin.Model = pin.Model()
        model.gravity.linear = np.array([0, -robot._g, 0])
        jid = 0
        pose = robot.base_pose
        for li, qi, jtype, mi in zip(
            robot.link_parameters, robot.q, robot.structure, robot.masses
        ):
            if jtype == "R":
                jid = model.addJoint(
                    jid, pin.JointModelRZ(), se2_to_pin_se3(pose), f"j{jid}"
                )
                pose = SE2(translation=[li, 0])
                body_inertia = pin.Inertia.Identity()
                body_inertia.mass = mi
                body_inertia.inertia = np.zeros((3, 3))
                model.appendBodyToJoint(jid, body_inertia, se2_to_pin_se3(pose))
            elif jtype == "P":
                pose *= SE2(rotation=SO2(li))
                jid = model.addJoint(
                    jid, pin.JointModelPX(), se2_to_pin_se3(pose), f"j{jid}"
                )
                pose = SE2()
                body_inertia = pin.Inertia.Identity()
                body_inertia.mass = mi
                body_inertia.inertia = np.zeros((3, 3))
                model.appendBodyToJoint(jid, body_inertia, se2_to_pin_se3(pose))

        model.addFrame(pin.Frame("flange", jid, 0, se2_to_pin_se3(pose), pin.OP_FRAME))
        return model

    def test_imported_modules(self):
        """Test that you are not using pinocchio inside your implementation."""
        with open(inspect.getfile(PlanarManipulatorDynamics)) as f:
            self.assertTrue("pinocchio" not in f.read())
        with open(inspect.getfile(PlanarManipulatorDynamics)) as f:
            self.assertTrue("cv2" not in f.read())
        with open(inspect.getfile(PlanarManipulatorDynamics)) as f:
            self.assertTrue("scipy" not in f.read())


if __name__ == "__main__":
    unittest.main()
