#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2024-10-21
#     Author: Martin Cífka <martin.cifka@cvut.cz>
#
from pathlib import Path
import cv2
from perception_utils import max_resize
from matplotlib import pyplot as plt


def main():
    img = cv2.imread(str(Path(__file__).parent / "aruco.jpg"))
    img = max_resize(img, max_width=1024)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(aruco_dict)

    # Detect markers and draw them
    corners, ids, rejected = detector.detectMarkers(gray)
    img_corners = img.copy()
    cv2.aruco.drawDetectedMarkers(img_corners, corners, ids)
    plt.imshow(img_corners[:, :, ::-1])
    plt.title("Detected markers")
    plt.show()

    # Draw rejected detections
    img_rejected = img.copy()
    cv2.aruco.drawDetectedMarkers(img_rejected, rejected, borderColor=(0, 0, 255))
    plt.imshow(img_rejected[:, :, ::-1])
    plt.title("Rejected detections")
    plt.show()


if __name__ == "__main__":
    main()
