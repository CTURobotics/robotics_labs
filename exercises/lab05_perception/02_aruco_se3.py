#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2024-10-21
#     Author: Martin Cífka <martin.cifka@cvut.cz>
#
# pip install opencv-contrib-python
from pathlib import Path
import cv2
import numpy as np
from matplotlib import pyplot as plt
from perception_utils import max_resize


def estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    """
    Source: https://stackoverflow.com/a/76802895
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    """
    marker_points = np.array(
        [
            [-marker_size / 2, marker_size / 2, 0],
            [marker_size / 2, marker_size / 2, 0],
            [marker_size / 2, -marker_size / 2, 0],
            [-marker_size / 2, -marker_size / 2, 0],
        ],
        dtype=np.float32,
    )
    trash = []
    rvecs = []
    tvecs = []
    for c in corners:
        nada, R, t = cv2.solvePnP(
            marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE
        )
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash


def main():
    # Load calibration saved in 02_calibration.py
    calibration = np.load("calibration.npz")
    K = calibration["K"]
    distortion = calibration["dist"]
    img_max_width = calibration["max_width"]

    # Load image and preprocess
    frame = cv2.imread(str(Path(__file__).parent / "aruco.jpg"))
    frame = max_resize(frame, max_width=img_max_width)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect makers
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(aruco_dict)
    corners, ids, rejected = detector.detectMarkers(gray)

    # Draw markers
    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    plt.imshow(frame[:, :, ::-1])
    plt.title("Detected markers")
    plt.show()

    # Estimate SE3 pose of each marker
    rvecs, tvecs, _ = estimatePoseSingleMarkers(corners, 0.03, K, distortion)
    for rvec, tvec in zip(rvecs, tvecs):
        cv2.drawFrameAxes(frame, K, distortion, rvec, tvec, 0.04)
        print(f"R = {rvec.T[0]}   |   t = {tvec.T[0]}")

    plt.imshow(frame[:, :, ::-1])
    plt.title("Markers with frames")
    plt.show()


if __name__ == "__main__":
    main()
