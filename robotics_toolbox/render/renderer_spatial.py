#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-7
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import time
from pathlib import Path

import numpy as np
from robomeshcat import Scene, Object

from robotics_toolbox.core import SE3, SO3
from robotics_toolbox.robots.drone import Drone


class RendererSpatial(Scene):
    def __init__(self) -> None:
        super().__init__()
        self.drones: dict[Drone, Object] = {}

    def __del__(self):
        time.sleep(1.0)

    @staticmethod
    def wait_for_enter(msg: str | None = None):
        if msg is None:
            msg = "Press enter to continue."
        input(msg)

    def plot_drone(self, robot: Drone):
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
        self.render()

    @staticmethod
    def _se3_to_meshcat_pose(pose: SE3):
        p = np.eye(4)
        p[:3, :3] = pose.rotation.rot
        p[:3, 3] = pose.translation
        return p
