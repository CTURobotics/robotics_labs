#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-22
#     Author: David Kovar <kovarda8@fel.cvut.cz>
#
import unittest
from pathlib import Path
import pickle
import inspect
import numpy as np
import pinocchio as pin
from robotics_toolbox.robots import PlanarManipulator


def check_if_identity(pose1: pin.SE3, pose2: pin.SE3, error: float = 0.001) -> bool:
    """check if transformation from pose1 to pose2 is identity with given error"""
    pose_2_1 = pose2.inverse() * pose1
    return pose_2_1.isIdentity(prec=error)


class TestSpatialURDF(unittest.TestCase):
    def test_poses_of_links_frames(self):
        """test whether the poses of links are similar to the reference ones"""
        np.random.seed(0)
        # making pinocchio model for testing
        path = Path(inspect.getfile(PlanarManipulator)).parent / "robot_hw.urdf"
        student_mod, _, _ = pin.buildModelsFromUrdf(str(path))
        student_data = student_mod.createData()

        # load model which will be used to check the solution
        path = Path(__file__).parent / "pin_mod_pin_data.pickle"
        with open(path, "rb") as file:
            data = pickle.load(file)
        pin_mod = data["mod"]
        pin_data = data["data"]

        # check 100 rand configurations with corespondence model
        for i in range(100):
            c = pin.randomConfiguration(pin_mod)
            pin.forwardKinematics(student_mod, student_data, c)
            pin.updateFramePlacements(student_mod, student_data)

            pin.forwardKinematics(pin_mod, pin_data, c)
            pin.updateFramePlacements(pin_mod, pin_data)
            self.assertTrue(
                check_if_identity(pin_data.oMf[-1], student_data.oMf[-1]),
                msg=f"Configuration {i} failed. \n frame {i} should be \n "
                f"{pin_data.oMf[-1].homogeneous} \n but is \n "
                f"{student_data.oMf[-1].homogeneous}",
            )


if __name__ == "__main__":
    unittest.main()
