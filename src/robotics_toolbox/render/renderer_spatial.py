#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-7
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from __future__ import annotations

import time
import webbrowser
from contextlib import contextmanager
from pathlib import Path

import numpy as np
import trimesh
import viser
import viser.transforms as vtf
from viser.extras import ViserUrdf

from robotics_toolbox.core import SE3, SO3
from robotics_toolbox.robots import Drone, SpatialManipulator


class RendererSpatial:
    """Renderer of 3D scenes based on viser. The scene is displayed in a web browser
    that connects to the local viser server started by this class."""

    def __init__(
        self, open: bool = True, wait_for_open: bool = True, port: int = 8080
    ) -> None:
        """Start the viser server. If @param open is True, the browser is opened
        automatically. If @param wait_for_open is True, wait until the browser connects
        to the server before continuing."""
        super().__init__()
        self.server = self._start_server(port)
        self.server.scene.set_up_direction("+z")
        self.server.scene.add_grid("/grid", width=4.0, height=4.0, plane="xy")

        self.drones: dict[Drone, viser.GlbHandle] = {}
        self.manipulators: dict[SpatialManipulator, tuple[viser.FrameHandle, ViserUrdf]]
        self.manipulators = {}
        self.poses: dict[SE3, viser.FrameHandle] = {}

        " Variables used internally in case we are rendering an animation "
        self._animation_fps: float | None = None
        self._camera_zoom = 1.0
        self.server.on_client_connect(self._on_client_connect)

        url = f"http://localhost:{self.server.get_port()}"
        print(f"Renderer is running at {url}")
        if open:
            webbrowser.open(url)
        if wait_for_open:
            print("Waiting for the browser to connect...")
            while len(self.server.get_clients()) == 0:
                time.sleep(0.1)

    @staticmethod
    def _start_server(port: int, attempts: int = 20) -> viser.ViserServer:
        """Start viser server on the first free port starting from @param port."""
        for p in range(port, port + attempts):
            try:
                return viser.ViserServer(port=p, verbose=False)
            except OSError:
                continue
        raise RuntimeError(f"No free port in range {port}-{port + attempts}.")

    @staticmethod
    def wait_for_enter(msg: str | None = None):
        if msg is None:
            msg = "Press enter to continue."
        input(msg)

    def wait_at_the_end(self):
        """A method that just sleep for a few seconds. Call it at the end to keep the
        renderer alive so that the browser can display the final scene."""
        time.sleep(10.0)

    """=== Animation ==="""

    @contextmanager
    def animation(self, fps: int = 30):
        """Context that plays the scene updates as an animation with a given frame
        rate, i.e. each render() call inside the context is displayed for 1/fps seconds.
        Usage:
            with renderer.animation(fps=30):
                renderer.plot_drone(drone)  # the first frame
                drone.pose = ...
                renderer.plot_drone(drone)  # the next frame
        """
        self._animation_fps = fps
        try:
            yield self
        finally:
            self._animation_fps = None

    def render(self):
        """Render the current scene. Viser displays the changes immediately, therefore
        this function only waits for the next frame if we are inside the animation."""
        if self._animation_fps is not None:
            time.sleep(1.0 / self._animation_fps)

    def render_image(self, height: int = 720, width: int = 1280) -> np.ndarray:
        """Render the current scene into an image using the camera of the connected
        browser."""
        clients = self.server.get_clients()
        if len(clients) == 0:
            raise RuntimeError("No browser is connected, cannot render image.")
        client = clients[min(clients.keys())]
        return client.get_render(height=height, width=width)

    """=== Camera control ==="""

    @property
    def camera_zoom(self) -> float:
        return self._camera_zoom

    @camera_zoom.setter
    def camera_zoom(self, zoom: float):
        """Zoom in (>1) or out (<1) the cameras of all connected browsers."""
        for client in self.server.get_clients().values():
            self._apply_zoom(client, zoom / self._camera_zoom)
        self._camera_zoom = zoom

    def _on_client_connect(self, client: viser.ClientHandle):
        self._apply_zoom(client, self._camera_zoom)

    @staticmethod
    def _apply_zoom(client: viser.ClientHandle, zoom: float):
        """Move camera towards the point it is looking at by a factor of zoom."""
        look_at = np.asarray(client.camera.look_at)
        position = np.asarray(client.camera.position)
        client.camera.position = look_at + (position - look_at) / zoom

    """=== Plotting ==="""

    def plot_drone(self, robot: Drone, render=True):
        vis_pose = SE3(
            rotation=SO3.exp([0, 0, -np.pi / 2]) * SO3.exp([np.pi / 2, 0, 0])
        )
        if robot in self.drones:
            wxyz, position = self._se3_to_viser(robot.pose * vis_pose)
            self.drones[robot].wxyz = wxyz
            self.drones[robot].position = position
        else:
            mesh = trimesh.load(Path(__file__).parent.joinpath("data/drone_costum.obj"))
            if isinstance(mesh, trimesh.Scene):
                mesh = mesh.dump(concatenate=True)
            mesh.visual = trimesh.visual.ColorVisuals(
                mesh, face_colors=[0.24 * 255, 0.24 * 255, 0.8 * 255, 255]
            )
            wxyz, position = self._se3_to_viser(robot.pose * vis_pose)
            self.drones[robot] = self.server.scene.add_mesh_trimesh(
                f"/drone_{len(self.drones)}",
                mesh,
                scale=0.1,
                wxyz=wxyz,
                position=position,
            )
        if render:
            self.render()

    def plot_manipulator(self, robot: SpatialManipulator, render=True):
        if robot not in self.manipulators:
            name = f"/manipulator_{len(self.manipulators)}"
            root = self.server.scene.add_frame(name, show_axes=False)
            self.manipulators[robot] = (
                root,
                ViserUrdf(self.server, robot.urdf, root_node_name=name),
            )
        root, urdf_visualization = self.manipulators[robot]
        root.wxyz, root.position = self._se3_to_viser(robot.base_pose)
        q = np.asarray(robot.q, dtype=float)
        urdf_visualization.update_cfg(dict(zip(robot.joint_names, q)))
        if render:
            self.render()

    def plot_se3(self, t: SE3, scale=1.0, render=True):
        wxyz, position = self._se3_to_viser(t)
        if t in self.poses:
            self.poses[t].wxyz = wxyz
            self.poses[t].position = position
        else:
            self.poses[t] = self.server.scene.add_frame(
                f"/se3_{len(self.poses)}",
                axes_length=0.25 * scale,
                axes_radius=0.01 * scale,
                wxyz=wxyz,
                position=position,
            )
        if render:
            self.render()

    @staticmethod
    def _se3_to_viser(pose: SE3) -> tuple[np.ndarray, np.ndarray]:
        """Convert SE3 to viser representation, i.e. quaternion (w,x,y,z) and position."""
        return vtf.SO3.from_matrix(pose.rotation.rot).wxyz, pose.translation
