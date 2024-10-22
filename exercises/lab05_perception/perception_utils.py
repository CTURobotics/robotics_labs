#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2024-10-21
#     Author: Martin Cífka <martin.cifka@cvut.cz>
#
import numpy as np
import matplotlib.pyplot as plt
import cv2
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def max_resize(img, max_width=1024, max_height=None):
    assert (max_width is None) != (max_height is None)

    if max_width is not None and img.shape[1] > max_width:
        s = max_width / img.shape[1]
        img = cv2.resize(img, (0, 0), fx=s, fy=s)

    elif max_height is not None and img.shape[0] > max_height:
        s = max_height / img.shape[0]
        img = cv2.resize(img, (0, 0), fx=s, fy=s)

    return img


def set_axes_equal(ax: plt.Axes):
    """Set 3D plot axes to equal scale.

    Make axes of 3D plot have equal scale so that spheres appear as
    spheres and cubes as cubes.  Required since `ax.axis('equal')`
    and `ax.set_aspect('equal')` don't work on 3D.
    """
    limits = np.array(
        [
            ax.get_xlim3d(),
            ax.get_ylim3d(),
            ax.get_zlim3d(),
        ]
    )
    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    _set_axes_radius(ax, origin, radius)


def _set_axes_radius(ax, origin, radius):
    x, y, z = origin
    ax.set_xlim3d([x - radius, x + radius])
    ax.set_ylim3d([y - radius, y + radius])
    ax.set_zlim3d([z - radius, z + radius])


class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def do_3d_projection(self, renderer=None):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))

        return np.min(zs)


def get_camera_pose(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    tvec = tvec.reshape(-1)
    camera_position = -R.T @ tvec
    camera_orientation = R.T

    return camera_position, camera_orientation


def viz_calibration(
    pts3d, grid, rvecs, tvecs, *, camera_idxs=None, axis_length=0.1, text_size=12
):
    if camera_idxs is None:
        camera_idxs = list(range(len(rvecs)))

    # Create a plot
    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection="3d")

    # Plot the 3D object points (checkerboard corners)
    # ax.scatter(pts3d[0,:,0], pts3d[0,:,1], pts3d[0,:,2], c='tab:orange', marker='o', label="Chessboard points")
    pts3d = pts3d.reshape(grid[1], grid[0], 3)
    for i in range(0, pts3d.shape[0] - 1):
        for j in range(0, pts3d.shape[1] - 1):
            idxs = [(i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)]
            rows, cols = np.array(list(zip(*idxs)))
            verts = pts3d[rows, cols][None]
            sq = Poly3DCollection(verts, alpha=0.3)
            sq.set_facecolor("black" if (i + j) % 2 else "white")
            sq.set_edgecolor("black")
            ax.add_collection3d(sq)

    # Plot camera positions and orientations
    for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
        cam_pos, cam_rot = get_camera_pose(rvec, tvec)

        # Plot the camera center (position)
        ax.scatter(
            cam_pos[0],
            cam_pos[1],
            cam_pos[2],
            c="black",
            marker="o",
            label="Camera centers" if i == 0 else "",
        )

        # Visualize the camera orientation as an axis (draw lines for the camera orientation axes)
        xyz_axes = (
            cam_rot.T * axis_length + cam_pos
        )  # Axes - vectors along the camera's x, y, z
        for axis, label, color in zip(
            xyz_axes, ["X Right", "Y Down", "Z Forward"], ["r", "g", "b"]
        ):
            ax.plot(
                [cam_pos[0], axis[0]],
                [cam_pos[1], axis[1]],
                [cam_pos[2], axis[2]],
                c=color,
                label=label if i == 0 else None,
            )
            # ax.add_artist(([cam_pos[0], axis[0]], [cam_pos[1], axis[1]], [cam_pos[2], axis[2]], label=label if i == 0 else None, mutation_scale=10, lw=1, arrowstyle="-|>", color=color))

        ax.text(
            cam_pos[0],
            cam_pos[1],
            cam_pos[2],
            str(camera_idxs[i]),
            size=text_size,
            zorder=1,
            color="k",
        )

    # Set plot labels
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    ax.set_box_aspect([1, 1, 1])
    set_axes_equal(ax)

    # Show the plot
    plt.legend()
    plt.show()
