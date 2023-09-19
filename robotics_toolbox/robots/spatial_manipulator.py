#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-08-21
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from pathlib import Path
from robomeshcat import Robot
import numpy as np
import pinocchio as pin

from robotics_toolbox.core import SE3, SO3


class SpatialManipulator:
    def __init__(
        self,
        robot_name: str | None = None,
        urdf_path: str | Path | None = None,
        mesh_folder_path: Path | str | list[Path] | list[str] | None = None,
        base_pose: SE3 | None = None,
        **kwargs
    ) -> None:
        """
        base_pose: where is the robot base placed
        robot_name: needs to be from the list: None, Panda, Talos, Tiago
        urdf_path and mesh_folder_path needs to be specified in robot_name is None
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

        self.meshcat_robot = Robot(
            urdf_path=urdf_path, mesh_folder_path=mesh_folder_path, **kwargs
        )
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
        pin.updateFramePlacements(self.meshcat_robot._model, self.meshcat_robot._data)
        frame_id = self.meshcat_robot._model.getFrameId(flange_link_name)
        m = self.meshcat_robot._data.oMf[frame_id].homogeneous
        return SE3(m[:3, 3], SO3(rotation_matrix=m[:3, :3]))

    def jacobian(self, flange_link_name: str | None = None) -> np.ndarray:
        """Computes jacobian of the manipulator for the given structure and
        configuration."""
        flange_link_name = self._resolve_flange_link_name(flange_link_name)
        fid = self.meshcat_robot._model.getFrameId(flange_link_name)
        return pin.computeFrameJacobian(
            self.meshcat_robot._model,
            self.meshcat_robot._data,
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
        assert self.meshcat_robot._model.existFrame(flange_link_name)
        return flange_link_name
