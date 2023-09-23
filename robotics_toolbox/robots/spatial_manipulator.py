#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-08-21
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from __future__ import annotations
from pathlib import Path
from robomeshcat import Robot
import numpy as np
import pinocchio as pin

from robotics_toolbox.core import SE3, SO3, SE2
from robotics_toolbox.robots.robot_base import RobotBase


class SpatialManipulator(RobotBase):
    def __init__(
        self,
        robot_name: str | None = None,
        urdf_path: str | Path | None = None,
        mesh_folder_path: Path | str | list[Path] | list[str] | None = None,
        srdf_path: Path | str | None = None,
        base_pose: SE3 | None = None,
        **kwargs,
    ) -> None:
        """
        base_pose: where is the robot base placed
        robot_name: needs to be from the list: None, Panda, Talos, Tiago
        urdf_path and mesh_folder_path needs to be specified in robot_name is None
        srdf_path: path to srdf that disable collisions
        """
        super().__init__()
        self.robot_name = robot_name
        if isinstance(robot_name, str):
            if robot_name.lower() == "panda":
                from example_robot_data.robots_loader import PandaLoader as RLoader
            elif robot_name.lower() == "talos":
                from example_robot_data.robots_loader import TalosLoader as RLoader
            elif robot_name.lower() == "tiago":
                from example_robot_data.robots_loader import TiagoDualLoader as RLoader
            else:
                raise NotImplementedError("Unknown robot.")

            urdf_path = RLoader().df_path
            mesh_folder_path = Path(RLoader().model_path).parent.parent
            srdf_path = RLoader().srdf_path

        self.meshcat_robot = Robot(
            urdf_path=urdf_path, mesh_folder_path=mesh_folder_path, **kwargs
        )

        (
            self._model,
            self._data,
            self._geom_model,
            self._geom_data,
        ) = self.meshcat_robot._build_model_from_urdf(urdf_path, mesh_folder_path, True)
        self._geom_model.addAllCollisionPairs()
        if srdf_path is not None:
            pin.removeCollisionPairs(self._model, self._geom_model, srdf_path)
        self._geom_data = self._geom_model.createData()

        self.base_pose = base_pose if base_pose is not None else SE3()
        self.q = np.zeros(len(self.meshcat_robot._q))

    @property
    def dof(self) -> int:
        """Return number of degrees of freedom for the robot."""
        return len(self.meshcat_robot._q)

    def flange_pose(self, flange_link_name: str | None = None) -> SE3:
        """Return a flange pose defined by the link name. Flange link name can be
        empty for Panda robot."""
        flange_link_name = self._resolve_flange_link_name(flange_link_name)
        self.meshcat_robot[:] = self.q
        pin.updateFramePlacements(self._model, self._data)
        frame_id = self._model.getFrameId(flange_link_name)
        m = self._data.oMf[frame_id].homogeneous
        return SE3(m[:3, 3], SO3(rotation_matrix=m[:3, :3]))

    def jacobian(self, flange_link_name: str | None = None) -> np.ndarray:
        """Computes jacobian of the manipulator for the given structure and
        configuration."""
        flange_link_name = self._resolve_flange_link_name(flange_link_name)
        fid = self._model.getFrameId(flange_link_name)
        return pin.computeFrameJacobian(
            self._model,
            self._data,
            self.q,
            fid,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )

    def _resolve_flange_link_name(self, flange_link_name: str | None = None) -> str:
        """Resolve flange link name. Use known for known robots if None, raise error
        otherwise."""
        if flange_link_name is None:
            assert self.robot_name is not None, "You need to specify flange_link_name"
            if self.robot_name.lower() == "panda":
                return "panda_link8"
            else:
                assert False, "You need to specify flange_link_name"
        assert self._model.existFrame(flange_link_name)
        return flange_link_name

    def sample_configuration(self) -> np.ndarray | SE2 | SE3:
        return pin.randomConfiguration(self._model)

    def set_configuration(self, configuration: np.ndarray | SE2 | SE3):
        self.q = configuration
        return self

    def in_collision(self) -> bool:
        return pin.computeCollisions(
            self._model,
            self._data,
            self._geom_model,
            self._geom_data,
            self.configuration(),
            True,
        )

    def configuration(self) -> np.ndarray | SE2 | SE3:
        return self.q
