#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-10-20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
# pip install opencv-python
import numpy as np
from robotics_toolbox.core import SE3, SO3
import cv2


def hand_eye(a: list[SE3], b: list[SE3]) -> tuple[SE3, SE3]:
    """Solve A^iX=YB^i, return X, Y"""

    rvec_a = [T.rotation.log() for T in a]
    tvec_a = [T.translation for T in a]
    rvec_b = [T.rotation.log() for T in b]
    tvec_b = [T.translation for T in b]

    Rx, tx, Ry, ty = cv2.calibrateRobotWorldHandEye(rvec_a, tvec_a, rvec_b, tvec_b)
    return SE3(tx[:, 0], SO3(Rx)), SE3(ty[:, 0], SO3(Ry))


np.random.seed(1)

gt_T_gc = SE3(translation=[0.05, 0.1, 0.2], rotation=SO3.exp([np.pi / 7, 0, 0]))
gt_T_rt = SE3(translation=[1.05, 1.1, 1.2], rotation=SO3.exp([np.pi / 3, 0.3, 0.2]))

T_rgs = []
T_cts = []
for i in range(100):
    T_rg = SE3(
        translation=np.random.uniform(-1, 1, size=3),
        rotation=SO3.exp(np.random.uniform(-np.pi, np.pi, size=3)),
    )
    T_ct = gt_T_gc.inverse() * T_rg.inverse() * gt_T_rt
    T_rgs.append(T_rg)
    T_cts.append(T_ct)


T_tr, T_cg = hand_eye(T_cts, [T.inverse() for T in T_rgs])
print(T_tr.inverse())
print(gt_T_rt)
print("---")
print(T_cg.inverse())
print(gt_T_gc)
