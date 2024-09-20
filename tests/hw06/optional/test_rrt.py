#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-22
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import inspect
import unittest
import numpy as np
from shapely import MultiPolygon, Point

from robotics_toolbox.core import SE2
from robotics_toolbox.planning.rrt import RRT
from robotics_toolbox.robots import PlanarManipulator
from robotics_toolbox.utils import distance_between_configurations


class TestRRT(unittest.TestCase):
    def test_plan_simple(self):
        np.random.seed(0)
        robot = PlanarManipulator(
            link_parameters=[0.3] * 5,
            base_pose=SE2([-0.75, 0.0]),
            structure="RRRRR",
        )
        robot.obstacles = MultiPolygon([Point((0.5, 0.5)).buffer(0.3, cap_style=3)])

        start_state = -np.pi / 4 * np.ones(robot.dof)
        goal_state = np.pi / 4 * np.ones(robot.dof)

        rrt = RRT(robot=robot, delta_q=0.2, p_sample_goal=0.5)
        path = rrt.plan(start_state, goal_state)
        self.assertGreater(len(path), 1)  # at least start and goal
        self.assertTrue(np.linalg.norm(start_state - path[0]) < 1e-5)
        self.assertTrue(np.linalg.norm(goal_state - path[-1]) < 1e-5)

        for p1, p2 in zip(path, path[1:]):
            self.assertLessEqual(distance_between_configurations(p1, p2), 0.21)
            self.assertFalse(robot.set_configuration(p1).in_collision())

        total_dist = sum(
            [distance_between_configurations(p1, p2) for p1, p2 in zip(path, path[1:])]
        )

        # test random shortcut
        path = rrt.random_shortcut(path, max_iterations=100)
        for p1, p2 in zip(path, path[1:]):
            self.assertLessEqual(distance_between_configurations(p1, p2), 0.21)
            self.assertFalse(robot.set_configuration(p1).in_collision())
        total_dist_simplified = sum(
            [distance_between_configurations(p1, p2) for p1, p2 in zip(path, path[1:])]
        )
        self.assertLess(total_dist_simplified, total_dist)

    def test_imported_modules(self):
        """Test that you are not using pinocchio inside your implementation."""
        with open(inspect.getfile(RRT)) as f:
            self.assertTrue("pinocchio" not in f.read())
        with open(inspect.getfile(RRT)) as f:
            self.assertTrue("cv2" not in f.read())
        with open(inspect.getfile(RRT)) as f:
            self.assertTrue("scipy" not in f.read())


if __name__ == "__main__":
    unittest.main()
