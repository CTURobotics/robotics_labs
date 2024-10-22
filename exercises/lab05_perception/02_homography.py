#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2024-10-21
#     Author: Martin Cífka <martin.cifka@cvut.cz>
#
from pathlib import Path
import cv2
import numpy as np
from perception_utils import max_resize
from matplotlib import pyplot as plt


def to_homogeneous(x):
    return np.array([x[0], x[1], 1.0])


def from_homogeneous(x):
    return x[:2] / x[2]


def main():
    img = cv2.imread(str(Path(__file__).parent / "aruco.jpg"))
    img = max_resize(img, max_width=1024)
    frame = img.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(aruco_dict)
    corners, ids, rejected = detector.detectMarkers(gray)

    n_markers = 4
    corners = np.array(corners).reshape(n_markers, 4, 2)

    # Sort the corners by the detected ids
    sort = np.argsort(ids.flatten())
    ids = ids[sort]
    corners = corners[sort]

    # Draw the 4 corners of the first detected marker
    plt.imshow(img[:, :, ::-1])
    for i, c in zip(range(4), ["r", "g", "b", "y"]):
        plt.scatter(*corners[0, i], c=c)
    plt.show()

    # Estimate the homography
    # (using only the 4 corners of the first detected marker -> not precise)
    src = np.array([[10, 10], [40, 10], [40, 40], [10, 40]], dtype=np.float32)
    dst = corners[0]
    H, _ = cv2.findHomography(src, dst)
    # TODO: 1. Get better estimate of H by using different points

    # Project the corners of the A4 paper to the image
    a_paper = to_homogeneous([0, 0])  # homogeneous coordinates on the A4 paper
    a_image = from_homogeneous(H @ a_paper)  # projective transformation

    b_paper = to_homogeneous([297, 210])
    b_image = from_homogeneous(H @ b_paper)

    plt.imshow(img[:, :, ::-1])
    plt.scatter(*a_image, c="r")
    plt.scatter(*b_image, c="g")
    plt.show()

    # TODO: 2. Use the homography H to find the coordinates of the point
    # on the paper [mm], in the coordinate system printed on the paper
    #
    # A4 paper - 297x210 mm
    # Margin - 10 mm
    # Marker size - 30 mm


if __name__ == "__main__":
    main()
