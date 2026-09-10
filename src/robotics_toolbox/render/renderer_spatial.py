#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-7
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from __future__ import annotations

import threading
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

    PLAYBACK_HZ = 60.0

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
        self._manipulator_q: dict[SpatialManipulator, np.ndarray] = {}

        " Variables used internally for recording and playing the animation "
        self._recording: list[dict] | None = None
        self._playback: _AnimationPlayback | None = None
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

    def wait_at_the_end(self, duration: float = 3.0):
        """Sleep for @param duration seconds to give the viewer time to show the scene,
        e.g. as a pause in the middle of the script. If an animation was recorded
        before, additionally keep the renderer running as long as a browser is connected
        so that the animation can be replayed from the browser; this returns a few
        seconds after the last browser disconnects, or on Ctrl+C. The browser connection
        itself is awaited already in the constructor (wait_for_open)."""
        time.sleep(duration)
        if self._playback is not None:
            self._keep_alive_while_connected()

    def _keep_alive_while_connected(self):
        """Block as long as a browser is connected to the server. Returns immediately if
        no browser is connected, a few seconds after the last one disconnects, or on
        Ctrl+C."""
        if len(self.server.get_clients()) == 0:
            return
        print("Animation can be replayed in the browser; close the tab to finish.")
        last_seen = time.time()
        try:
            while time.time() - last_seen < 3.0:
                if len(self.server.get_clients()) > 0:
                    last_seen = time.time()
                time.sleep(0.5)
        except KeyboardInterrupt:
            return

    """=== Animation ==="""

    @contextmanager
    def animation(self, fps: int = 30):
        """Context that records an animation: each render() call inside the context
        stores one frame of the animation. When the context is closed, the animation is
        played in the browser with a given frame rate and can be controlled (play,
        pause, loop, seek) from the 'Animation' panel of the viewer. The frames are
        interpolated during the playback, so the motion is smooth even for low fps.
        Usage:
            with renderer.animation(fps=30):
                renderer.plot_drone(drone)  # the first frame
                drone.pose = ...
                renderer.plot_drone(drone)  # the next frame
        """
        self._stop_playback()
        self._recording = []
        try:
            yield self
        finally:
            frames, self._recording = self._recording, None
            if len(frames) > 1:
                self._playback = _AnimationPlayback(self, frames, fps)

    def render(self):
        """Render the current scene. Viser displays the changes immediately, therefore
        this function only records the frame if we are inside the animation."""
        if self._recording is not None:
            self._recording.append(self._snapshot())

    def render_image(self, height: int = 720, width: int = 1280) -> np.ndarray:
        """Render the current scene into an image using the camera of the connected
        browser."""
        clients = self.server.get_clients()
        if len(clients) == 0:
            raise RuntimeError("No browser is connected, cannot render image.")
        client = clients[min(clients.keys())]
        return client.get_render(height=height, width=width)

    def _snapshot(self) -> dict:
        """Store poses of all plotted objects; key is the plotted object (SE3, Drone or
        SpatialManipulator), value is (wxyz, position[, joint values])."""
        snapshot = {}
        for obj, handle in [*self.poses.items(), *self.drones.items()]:
            snapshot[obj] = (np.array(handle.wxyz), np.array(handle.position))
        for robot, (root, _) in self.manipulators.items():
            snapshot[robot] = (
                np.array(root.wxyz),
                np.array(root.position),
                self._manipulator_q[robot].copy(),
            )
        return snapshot

    def _apply_snapshot(self, snapshot: dict):
        """Set poses of all plotted objects from the snapshot; objects that were not
        plotted at the time of the snapshot are hidden."""
        with self.server.atomic():
            for obj, handle in [*self.poses.items(), *self.drones.items()]:
                if obj in snapshot:
                    handle.wxyz, handle.position = snapshot[obj]
                handle.visible = obj in snapshot
            for robot, (root, urdf_visualization) in self.manipulators.items():
                if robot in snapshot:
                    root.wxyz, root.position, q = snapshot[robot]
                    urdf_visualization.update_cfg(dict(zip(robot.joint_names, q)))
                root.visible = robot in snapshot

    def _stop_playback(self):
        if self._playback is not None:
            self._playback.stop()
            self._playback = None

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
        q = np.asarray(robot.q, dtype=float).copy()
        self._manipulator_q[robot] = q
        urdf_visualization.update_cfg(dict(zip(robot.joint_names, q)))
        if render:
            self.render()

    def plot_se3(self, t: SE3, scale=1.0, render=True):
        """Plot SE3 as a coordinate frame; @param scale is the length of the axes in
        meters (used only when the frame is plotted for the first time)."""
        wxyz, position = self._se3_to_viser(t)
        if t in self.poses:
            self.poses[t].wxyz = wxyz
            self.poses[t].position = position
        else:
            self.poses[t] = self.server.scene.add_frame(
                f"/se3_{len(self.poses)}",
                axes_length=scale,
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


class _AnimationPlayback:
    """Plays recorded frames in the browser with interpolation between the frames and
    exposes play/pause, seeking, looping and speed in the GUI of the viewer."""

    def __init__(self, renderer: RendererSpatial, frames: list[dict], fps: float):
        self.renderer = renderer
        self.frames = frames
        self.fps = fps
        self._time = 0.0  # position in the animation expressed in frames
        self._updating_gui = False
        self._stop_event = threading.Event()

        gui = renderer.server.gui
        self._folder = gui.add_folder("Animation")
        with self._folder:
            self._play = gui.add_checkbox("Play", initial_value=True)
            self._loop = gui.add_checkbox("Loop", initial_value=False)
            self._speed = gui.add_slider(
                "Speed", min=0.1, max=4.0, step=0.1, initial_value=1.0
            )
            self._frame = gui.add_slider(
                "Frame", min=0, max=len(frames) - 1, step=1, initial_value=0
            )

        @self._frame.on_update
        def _(_) -> None:
            if not self._updating_gui:
                self._time = float(self._frame.value)
                self._play.value = False

        @self._play.on_update
        def _(_) -> None:
            # playing again after the animation finished starts from the beginning
            if self._play.value and self._time >= len(self.frames) - 1:
                self._time = 0.0

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=1.0)
        self._folder.remove()

    def _run(self):
        last = time.perf_counter()
        last_frame_shown = -1
        while not self._stop_event.is_set():
            now = time.perf_counter()
            dt, last = now - last, now
            if self._play.value:
                self._time += dt * self.fps * self._speed.value
                end = len(self.frames) - 1
                if self._time >= end:
                    if self._loop.value:
                        self._time %= end
                    else:
                        self._time = end
                        self._play.value = False
            self.renderer._apply_snapshot(self._interpolated_frame(self._time))
            if int(round(self._time)) != last_frame_shown:
                last_frame_shown = int(round(self._time))
                self._updating_gui = True
                self._frame.value = last_frame_shown
                self._updating_gui = False
            time.sleep(1.0 / RendererSpatial.PLAYBACK_HZ)

    def _interpolated_frame(self, t: float) -> dict:
        """Interpolate between recorded frames floor(t) and ceil(t)."""
        i = int(np.floor(t))
        alpha = t - i
        a = self.frames[min(i, len(self.frames) - 1)]
        b = self.frames[min(i + 1, len(self.frames) - 1)]
        if alpha < 1e-6 or a is b:
            return a
        frame = {}
        for key, state_a in a.items():
            if key not in b:
                frame[key] = state_a
                continue
            state_b = b[key]
            wxyz = self._slerp(state_a[0], state_b[0], alpha)
            position = (1 - alpha) * state_a[1] + alpha * state_b[1]
            if len(state_a) == 3:  # manipulator: interpolate also the joint values
                frame[key] = (
                    wxyz,
                    position,
                    (1 - alpha) * state_a[2] + alpha * state_b[2],
                )
            else:
                frame[key] = (wxyz, position)
        return frame

    @staticmethod
    def _slerp(wxyz_a: np.ndarray, wxyz_b: np.ndarray, alpha: float) -> np.ndarray:
        ra, rb = vtf.SO3(wxyz_a), vtf.SO3(wxyz_b)
        return ra.multiply(vtf.SO3.exp(alpha * ra.inverse().multiply(rb).log())).wxyz
