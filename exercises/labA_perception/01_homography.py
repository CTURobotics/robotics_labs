#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-10-23
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import matplotlib.pyplot as plt
import numpy as np
from robotics_toolbox.core import SE3, SO3

# Additional dependency required for this script!
# pip install opencv-python
import cv2

n = 8
xyz_r = np.random.uniform(-1, 1, size=(n, 3))
xyz_r[:, 2] = 0

# T_RC = SE3(translation=[0.1, 0, 1.]) * SE3(rotation=SO3.rx(np.pi))
T_RC = SE3(translation=[0.1, 0, 1.0], rotation=SO3.exp([np.pi, 0, 0]))
T_CR = T_RC.inverse()
xyz_c = np.asarray([T_CR.act(x) for x in xyz_r])

fig: plt.Figure = plt.figure()
ax_spatial: plt.Axes = fig.add_subplot(121, projection="3d")
ax_spatial.plot(xyz_r[:, 0], xyz_r[:, 1], xyz_r[:, 2], "o", ms=10, color="tab:blue")
# ax_spatial.plot(*xyz_r.T, 'o', ms=10, color='tab:blue')
ax_spatial.set_xlabel("x [m]")
ax_spatial.set_ylabel("y [m]")
ax_spatial.set_zlabel("z [m]")
ax_spatial.plot(*T_RC.translation, "^k", ms=10)
# ax_spatial.plot(T_RC.translation[0], T_RC.translation[1], T_RC.translation[2], '^k', ms=10)

ax_image: plt.Axes = fig.add_subplot(122)
ax_image.set_xlabel("u [px]")
ax_image.set_ylabel("v [px]")

K = np.array(
    [
        [240, 0, 0],
        [0, 240, 0],
        [0, 0, 1],
    ]
)

uv = np.zeros((n, 2))
for i in range(n):
    u_h = K @ xyz_c[i]
    # uv[i, 0] = u_h[0] / u_h[2]
    # uv[i, 1] = u_h[1] / u_h[2]
    uv[i, :2] = u_h[:2] / u_h[2]

ax_image.plot(*uv.T, "o", ms=10, color="tab:blue")


H, _ = cv2.findHomography(uv[:, :2], xyz_r[:, :2])

xyz_r2 = np.zeros_like(xyz_r)
for i in range(n):
    x = H @ np.append(uv[i], [1])
    xyz_r2[i, :2] = x[:2] / x[2]

ax_spatial.plot(*xyz_r2.T, "x", ms=10)

plt.show()
