#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-22
#     Author: David Kovar <kovarda8@fel.cvut.cz>
#
import unittest
from pathlib import Path

import numpy as np
import pinocchio as pin


def check_if_identity(pose1: pin.SE3, pose2: pin.SE3, error: float = 0.001) -> bool:
    """check if transformation from pose1 to pose2 is identity with given error"""
    pose_2_1 = pose2.inverse() * pose1
    return pose_2_1.isIdentity(prec=error)


class TestSpatialURDF(unittest.TestCase):
    def test_number_of_frames(self):
        """test whether the robot has the correct amount of links and joints"""
        path = Path.joinpath(Path(__file__).parents[3], "exercises/lab02/robot_hw.urdf")
        pin_mod, col_mod, _ = pin.buildModelsFromUrdf(str(path))
        self.assertEqual(len(pin_mod.frames), 11)

    def test_poses_of_links_frames(self):
        """test whether the poses of links are similar to the reference ones"""
        # making pinocchio model for testing
        path = Path.joinpath(Path(__file__).parents[3], "exercises/lab02/robot_hw.urdf")
        pin_mod, col_mod, _ = pin.buildModelsFromUrdf(str(path))
        pin_data = pin_mod.createData()

        # configurations to try
        configurations = np.array(
            [
                [0, 0, 0],
                [-0.5, np.pi / 2, -np.pi / 2],
                [0.2, -np.pi / 4, -np.pi / 2],
                [0, np.pi, np.pi],
            ]
        )

        # correct reference poses of frames
        pose_o_i = [
            [
                pin.SE3(rotation=np.eye(3), translation=np.array([0, 0, 0])),
                pin.SE3(rotation=np.eye(3), translation=np.array([0, 0, 0.05])),
                pin.SE3(rotation=np.eye(3), translation=np.array([0, 0, 0.05])),
                pin.SE3(rotation=np.eye(3), translation=np.array([0, 0, 0.55])),
                pin.SE3(rotation=np.eye(3), translation=np.array([0, 0, 0.55])),
            ],
            [
                pin.SE3(rotation=np.eye(3), translation=np.array([0, 0, 0])),
                pin.SE3(rotation=np.eye(3), translation=np.array([0, -0.5, 0.05])),
                pin.XYZQUATToSE3([0, -0.5, 0.05] + [0, 0, 0.7071068, 0.7071068]),
                pin.XYZQUATToSE3([0, -0.5, 0.55] + [0, 0, 0.7071068, 0.7071068]),
                pin.XYZQUATToSE3([0, -0.5, 0.55] + [-0.5, -0.5, 0.5, 0.5]),
            ],
            [
                pin.SE3(rotation=np.eye(3), translation=np.array([0, 0, 0])),
                pin.SE3(rotation=np.eye(3), translation=np.array([0, 0.2, 0.05])),
                pin.XYZQUATToSE3([0, 0.2, 0.05] + [0, 0, -0.3826835, 0.9238795]),
                pin.XYZQUATToSE3([0, 0.2, 0.55] + [0, 0, -0.3826835, 0.9238795]),
                pin.XYZQUATToSE3(
                    [0, 0.2, 0.55] + [-0.6532815, 0.2705981, -0.2705981, 0.6532815]
                ),
            ],
            [
                pin.SE3(rotation=np.eye(3), translation=np.array([0, 0, 0])),
                pin.SE3(rotation=np.eye(3), translation=np.array([0, 0, 0.05])),
                pin.XYZQUATToSE3([0, 0, 0.05] + [0, 0, 1, 0]),
                pin.XYZQUATToSE3([0, 0, 0.55] + [0, 0, 1, 0]),
                pin.XYZQUATToSE3([0, 0, 0.55] + [0, 1, 0, 0]),
            ],
        ]

        for i, c in enumerate(configurations):
            pin.forwardKinematics(pin_mod, pin_data, c)
            pin.updateFramePlacements(pin_mod, pin_data)
            for k, j in enumerate(range(2, len(pin_mod.frames), 2)):
                # no need to check first two frames (universe link and root joint)
                # check only every second frame (robot is of type link - join - link...)
                self.assertTrue(check_if_identity(pin_data.oMf[j], pose_o_i[i][k]))


if __name__ == "__main__":
    unittest.main()
