#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-11-9
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
# pip install opencv-contrib-python
from pathlib import Path

import cv2
import numpy as np

frame = cv2.imread(str(Path(__file__).parent / "img.png"))
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
detector = cv2.aruco.ArucoDetector(aruco_dict)

# Detect markers and draw them
(corners, ids, rejected) = detector.detectMarkers(gray)
cv2.aruco.drawDetectedMarkers(frame, corners, ids)

cv2.imshow("Image with markers", frame)
cv2.waitKey()

# Estimate SE3 pose of the marker
camera_matrix = np.array(
    [
        [240.0, 0, 0],
        [0, 240, 0],
        [0, 0, 1],
    ]
)
distortion = np.zeros(5)
for i in range(len(ids)):
    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners[i], 0.04, camera_matrix, distCoeffs=distortion
    )
    cv2.drawFrameAxes(frame, camera_matrix, distortion, rvec, tvec, 0.04)

    print(tvec)

cv2.imshow("Image with frames", frame)
cv2.waitKey()
