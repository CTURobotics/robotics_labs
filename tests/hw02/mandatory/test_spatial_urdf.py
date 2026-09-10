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
import yourdfpy
from robotics_toolbox.robots import PlanarManipulator
from robotics_toolbox.utils.urdf_utils import chain_ordered_joint_names, leaf_link


def check_if_identity(
    pose1: np.ndarray, pose2: np.ndarray, error: float = 0.001
) -> bool:
    """check if transformation from pose1 to pose2 is identity with given error"""
    pose_2_1 = np.linalg.inv(pose2) @ pose1
    return np.allclose(pose_2_1, np.eye(4), atol=error)


class TestSpatialURDF(unittest.TestCase):
    def test_poses_of_links_frames(self):
        """test whether the poses of links are similar to the reference ones"""
        np.random.seed(0)
        # loading the student URDF, only kinematics is needed
        path = Path(inspect.getfile(PlanarManipulator)).parent / "robot_hw.urdf"
        student_urdf = yourdfpy.URDF.load(
            str(path), load_meshes=False, build_scene_graph=True
        )
        # configuration is interpreted in the order of the kinematic chain, the end
        # effector is the last link of the chain
        joint_names = chain_ordered_joint_names(student_urdf)
        end_effector = leaf_link(student_urdf)

        with open(Path(__file__).parent / "reference_poses.pickle", "rb") as file:
            data = pickle.load(file)
        configs = data["configs"]
        poses = data["poses"]
        for i, (c, ref_pose) in enumerate(zip(configs, poses)):
            self.assertEqual(
                len(joint_names),
                len(c),
                msg=f"Robot should have {len(c)} actuated joints, found {joint_names}.",
            )
            student_urdf.update_cfg(dict(zip(joint_names, c)))
            pose = student_urdf.get_transform(end_effector, student_urdf.base_link)
            self.assertTrue(
                check_if_identity(ref_pose, pose),
                msg=f"Configuration {c} failed. \n frame {i} should be \n "
                f"{ref_pose} \n but is \n {pose}",
            )


if __name__ == "__main__":
    unittest.main()
