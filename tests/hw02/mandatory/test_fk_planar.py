#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-18
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import unittest

import numpy as np

from robotics_toolbox.robots import PlanarManipulator
from tests.utils import (
    planar_fk_reference,
    assert_se2_equals_hom,
    sample_planar_manipulator,
    assert_no_forbidden_imports,
)


class TestFKPlanar(unittest.TestCase):

    def test_flange_pose(self):
        np.random.seed(0)
        for _ in range(100):
            robot = sample_planar_manipulator()
            robot.q = np.random.uniform(-np.pi, np.pi, size=robot.dof)
            ref_frames = planar_fk_reference(robot)
            assert_se2_equals_hom(self, robot.flange_pose(), ref_frames[-1])

    def test_fk_all_links(self):
        np.random.seed(0)
        for _ in range(100):
            robot = sample_planar_manipulator()
            robot.q = np.random.uniform(-np.pi, np.pi, size=robot.dof)

            ref_frames = planar_fk_reference(robot)

            frames = robot.fk_all_links()
            self.assertEqual(len(frames), robot.dof + 1)
            self.assertEqual(frames[0], robot.base_pose)

            for f, fref in zip(frames, ref_frames):
                assert_se2_equals_hom(self, f, fref)
            assert_se2_equals_hom(self, frames[-1], ref_frames[-1])

    def test_imported_modules(self):
        """Test that you are not using any external library (scipy, ...) inside your
        implementation, only python standard library and numpy are allowed."""
        assert_no_forbidden_imports(self, PlanarManipulator)


if __name__ == "__main__":
    unittest.main()
