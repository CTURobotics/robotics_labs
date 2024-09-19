#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-7
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from __future__ import annotations
import time
from pathlib import Path

import numpy as np
from robomeshcat import Scene, Object, Robot
import meshcat.geometry as g

from robotics_toolbox.core import SE3, SO3
from robotics_toolbox.robots import Drone, SpatialManipulator


class RendererSpatial(Scene):
    def __init__(self) -> None:
        super().__init__()
        self.drones: dict[Drone, Object] = {}
        self.manipulators: dict[SpatialManipulator, Robot] = {}
        self.poses: dict[SE3, Object] = {}

    @staticmethod
    def wait_for_enter(msg: str | None = None):
        if msg is None:
            msg = "Press enter to continue."
        input(msg)

    def wait_at_the_end(self):
        """A method that just sleep for a few seconds. Call it at the end to prevent
        interruption of the connection with the renderer."""
        time.sleep(10.0)

    def plot_drone(self, robot: Drone, render=True):
        vis_pose = SE3(
            rotation=SO3.exp([0, 0, -np.pi / 2]) * SO3.exp([np.pi / 2, 0, 0])
        )
        if robot in self.drones:
            self.drones[robot].pose = self._se3_to_meshcat_pose(robot.pose * vis_pose)
        else:
            self.drones[robot] = Object.create_mesh(
                path_to_mesh=Path(__file__).parent.joinpath("data/drone_costum.obj"),
                scale=0.1,
                pose=self._se3_to_meshcat_pose(robot.pose * vis_pose),
                color=[0.24, 0.24, 0.8],
            )
            self.add_object(self.drones[robot])
        if render:
            self.render()

    def plot_manipulator(self, robot: SpatialManipulator, render=True):
        if robot in self.manipulators:
            self.manipulators[robot].pose = self._se3_to_meshcat_pose(robot.base_pose)
            self.manipulators[robot][:] = robot.q[:]
        else:
            self.manipulators[robot] = robot.meshcat_robot
            self.add_robot(self.manipulators[robot])
            self.plot_manipulator(robot)
        if render:
            self.render()

    def plot_se3(self, t: SE3, scale=1.0, render=True):
        if t in self.poses:
            self.poses[t].pose = self._se3_to_meshcat_pose(t)
        else:
            self.poses[t] = Object(g.triad(scale), pose=self._se3_to_meshcat_pose(t))
            self.add_object(self.poses[t])
        if render:
            self.render()

    @staticmethod
    def _se3_to_meshcat_pose(pose: SE3):
        p = np.eye(4)
        p[:3, :3] = pose.rotation.rot
        p[:3, 3] = pose.translation
        return p
