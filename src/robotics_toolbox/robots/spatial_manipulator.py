#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-08-21
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from __future__ import annotations

import importlib
from functools import partial
from pathlib import Path

import numpy as np
import yourdfpy
from trimesh.collision import CollisionManager

from robotics_toolbox.core import SE3, SE2
from robotics_toolbox.robots.robot_base import RobotBase
from robotics_toolbox.utils.urdf_utils import (
    chain_ordered_joints,
    ancestor_joint_names,
    joint_limits,
    joint_axis,
    srdf_disabled_collision_pairs,
    hom_to_se3,
)

"""Robots that can be loaded by name; models are downloaded by robot_descriptions."""
KNOWN_ROBOTS = {
    "panda": dict(module="panda_description", flange="panda_link8"),
    "talos": dict(module="talos_description", flange=None),
    "tiago": dict(module="tiago_description", flange=None, urdf="URDF_PATH_DUAL"),
}


class SpatialManipulator(RobotBase):
    def __init__(
        self,
        robot_name: str | None = None,
        urdf_path: str | Path | None = None,
        mesh_folder_path: Path | str | None = None,
        srdf_path: Path | str | None = None,
        base_pose: SE3 | None = None,
        **kwargs,
    ) -> None:
        """
        base_pose: where is the robot base placed
        robot_name: needs to be from the list: None, Panda, Talos, Tiago
        urdf_path needs to be specified if robot_name is None
        mesh_folder_path: folder used to resolve mesh paths of the urdf, by default
          the folder of the urdf file (and its parents) is searched
        srdf_path: path to srdf that disable collisions between selected links
        kwargs: additional arguments passed to yourdfpy.URDF.load
        """
        super().__init__()
        self.robot_name = robot_name
        self._flange_link_name: str | None = None
        if isinstance(robot_name, str):
            urdf_path, known_srdf_path, mesh_folder_path = self._resolve_known_robot(
                robot_name
            )
            srdf_path = known_srdf_path if srdf_path is None else srdf_path
        assert urdf_path is not None, "Specify either robot_name or urdf_path."

        self.urdf: yourdfpy.URDF = self._load_urdf(
            urdf_path, mesh_folder_path, **kwargs
        )
        self._joints = chain_ordered_joints(self.urdf)
        self._disabled_collision_pairs = (
            srdf_disabled_collision_pairs(srdf_path) if srdf_path is not None else set()
        )
        self._collision_manager: CollisionManager | None = None

        self.base_pose = base_pose if base_pose is not None else SE3()
        self.q = np.zeros(self.dof)

    def _resolve_known_robot(self, robot_name: str):
        """Return urdf path, srdf path and mesh folder for a known robot."""
        known = KNOWN_ROBOTS.get(robot_name.lower())
        if known is None:
            raise NotImplementedError("Unknown robot.")
        module = importlib.import_module(f"robot_descriptions.{known['module']}")
        self._flange_link_name = known["flange"]
        urdf_path = getattr(module, known.get("urdf", "URDF_PATH"))
        package_path = Path(module.PACKAGE_PATH)
        srdf_path = getattr(module, "SRDF_PATH", None)
        if srdf_path is None:
            candidates = sorted(package_path.glob("srdf/*.srdf"))
            preferred = [c for c in candidates if c.stem == robot_name.lower()]
            srdf_path = (preferred or candidates or [None])[0]
        return urdf_path, srdf_path, package_path

    @staticmethod
    def _load_urdf(
        urdf_path: str | Path, mesh_folder_path: str | Path | None, **kwargs
    ) -> yourdfpy.URDF:
        """Load URDF including visual and collision geometry."""
        mesh_folder_path = (
            Path(urdf_path).parent if mesh_folder_path is None else mesh_folder_path
        )
        load_options = dict(
            build_scene_graph=True,
            load_meshes=True,
            build_collision_scene_graph=True,
            load_collision_meshes=True,
            filename_handler=partial(
                yourdfpy.filename_handler_magic, dir=str(mesh_folder_path)
            ),
        )
        load_options.update(kwargs)
        return yourdfpy.URDF.load(str(urdf_path), **load_options)

    @property
    def dof(self) -> int:
        """Return number of degrees of freedom for the robot."""
        return len(self._joints)

    @property
    def joint_names(self) -> list[str]:
        """Names of the actuated joints in the order of the configuration vector."""
        return [joint.name for joint in self._joints]

    def _update_fk(self):
        """Propagate the current configuration into the URDF kinematic tree."""
        q = np.asarray(self.q, dtype=float)
        assert q.shape == (self.dof,), f"Configuration needs {self.dof} elements."
        self.urdf.update_cfg({joint.name: qi for joint, qi in zip(self._joints, q)})

    def _link_pose(self, link_name: str) -> np.ndarray:
        """Homogeneous matrix of the link frame w.r.t. the base link. FK needs to be
        updated before."""
        return self.urdf.get_transform(link_name, self.urdf.base_link)

    def flange_pose(self, flange_link_name: str | None = None) -> SE3:
        """Return a flange pose defined by the link name. Flange link name can be
        empty for Panda robot."""
        flange_link_name = self._resolve_flange_link_name(flange_link_name)
        self._update_fk()
        return hom_to_se3(self._link_pose(flange_link_name))

    def link_poses(self) -> dict[str, SE3]:
        """Return poses of all link frames w.r.t. the base link for the current
        configuration."""
        self._update_fk()
        return {name: hom_to_se3(self._link_pose(name)) for name in self.urdf.link_map}

    def jacobian(self, flange_link_name: str | None = None) -> np.ndarray:
        """Computes 6xDoF geometric jacobian of the flange, i.e. linear and angular
        velocity of the flange origin expressed in the base frame."""
        flange_link_name = self._resolve_flange_link_name(flange_link_name)
        self._update_fk()
        flange_position = self._link_pose(flange_link_name)[:3, 3]
        ancestors = ancestor_joint_names(self.urdf, flange_link_name)

        jac = np.zeros((6, self.dof))
        for i, joint in enumerate(self._joints):
            if joint.name not in ancestors:
                continue  # joint does not move the flange
            joint_pose = self._link_pose(joint.child)
            axis = joint_pose[:3, :3] @ joint_axis(joint)
            if joint.type == "prismatic":
                jac[:3, i] = axis
            else:
                jac[:3, i] = np.cross(axis, flange_position - joint_pose[:3, 3])
                jac[3:, i] = axis
        return jac

    def _resolve_flange_link_name(self, flange_link_name: str | None = None) -> str:
        """Resolve flange link name. Use known for known robots if None, raise error
        otherwise."""
        if flange_link_name is None:
            assert (
                self._flange_link_name is not None
            ), "You need to specify flange_link_name"
            flange_link_name = self._flange_link_name
        assert (
            flange_link_name in self.urdf.link_map
        ), f"Unknown link {flange_link_name}"
        return flange_link_name

    def sample_configuration(self) -> np.ndarray | SE2 | SE3:
        """Sample configuration uniformly inside the joint limits."""
        limits = np.array([joint_limits(joint) for joint in self._joints])
        return np.random.uniform(limits[:, 0], limits[:, 1])

    def set_configuration(self, configuration: np.ndarray | SE2 | SE3):
        self.q = configuration
        return self

    def in_collision(self) -> bool:
        """Check self-collision of the robot using its collision geometry; pairs of
        links disabled in the SRDF are ignored."""
        scene = self.urdf.collision_scene
        assert scene is not None, "URDF was loaded without collision geometry."
        self._update_fk()

        if self._collision_manager is None:
            self._collision_manager = CollisionManager()
            for node in scene.graph.nodes_geometry:
                transform, geometry_name = scene.graph.get(node)
                self._collision_manager.add_object(
                    node, scene.geometry[geometry_name], transform=transform
                )
        else:
            for node in scene.graph.nodes_geometry:
                transform, _ = scene.graph.get(node)
                self._collision_manager.set_transform(node, transform)

        _, colliding_nodes = self._collision_manager.in_collision_internal(
            return_names=True
        )
        link_of_node = scene.graph.transforms.parents
        for node_a, node_b in colliding_nodes:
            links = frozenset((link_of_node[node_a], link_of_node[node_b]))
            if len(links) == 2 and links not in self._disabled_collision_pairs:
                return True
        return False

    def configuration(self) -> np.ndarray | SE2 | SE3:
        return self.q
