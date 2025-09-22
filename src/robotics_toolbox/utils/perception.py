#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2025-09-21
#     Author: Martin Cífka <martin.cifka@cvut.cz>
#
from typing import List
from numpy.typing import ArrayLike
import numpy as np
import cv2  # noqa


def find_hoop_homography(images: ArrayLike, hoop_positions: List[dict]) -> np.ndarray:
    """
    Find homography based on images containing the hoop and the hoop positions loaded from
    the hoop_positions.json file in the following format:

    [{
        "rotation_matrix": [-0.0005572332585040621, -3.141058227474627, 0.0005185830258253442],
        "translation_vector": [0.5093259019899434, -0.17564068853313258, 0.04918733225140541]
    },
    {
      "rotation_matrix": [-0.0005572332585040621, -3.141058227474627, 0.0005185830258253442],
      "translation_vector": [0.5093569397977782, -0.08814069881074972, 0.04918733225140541]
    },
    ...
    ]
    """

    images = np.asarray(images)
    assert images.shape[0] == len(hoop_positions)

    # TODO: 1. Detect circle in each image
    # TODO: 2. Find homography using cv2.findHomography. Use the hoop positions and circle centers.

    return find_hoop_homography_impl(images, hoop_positions)
    return find_hoop_homography_impl_contour(images, hoop_positions)
    return np.eye(3)


def find_hoop_homography_impl(
    images: ArrayLike,
    hoop_positions: List[dict],
    dp: float = 1.0,
    min_dist: float = 50.0,
    param1: float = 10.0,
    param2: float = 20.0,
    min_radius: int = 0,
    max_radius: int = 0,
) -> np.ndarray:

    ref_pts = np.array([x["translation_vector"] for x in hoop_positions])[:, :2]
    assert len(images) == len(ref_pts)

    img_pts = []
    img_circles = []
    for img in images:
        color_diffs = img - np.array([105, 75, 30]).reshape(1, 1, 3)
        color_dists = np.linalg.norm(color_diffs, axis=-1)
        mask = color_dists < 128
        mask = mask.astype(np.uint8) * 255

        circles = cv2.HoughCircles(
            (mask),
            cv2.HOUGH_GRADIENT,
            dp,
            minDist=min_dist,
            param1=param1,
            param2=param2,
            minRadius=min_radius,
            maxRadius=max_radius,
        )[0]
        circle = circles[0]
        center = circle[:2]
        img_pts.append(center)
        img_circles.append(circle)

    img_pts = np.array(img_pts)
    img_pts += np.random.normal(scale=4.0, size=img_pts.shape)

    H, _ = cv2.findHomography(img_pts, ref_pts)

    return H


def find_hoop_contour(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    best = None
    best_score = 0
    for c in contours:
        area = cv2.contourArea(c)
        if area < 100:
            continue
        perimeter = cv2.arcLength(c, True)
        if perimeter == 0:
            continue
        circularity = 4 * np.pi * area / (perimeter**2)
        score = circularity * np.log(area + 1)
        if score > best_score:
            best_score = score
            best = c
    if best is None:
        return None
    (x, y), radius = cv2.minEnclosingCircle(best)
    center = (int(x), int(y))
    radius = int(radius)
    return {"contour": best, "center": center, "radius": radius}


def find_hoop_homography_impl_contour(
    images: ArrayLike, hoop_positions: List[dict]
) -> np.ndarray:
    ref_pts = np.array([x["translation_vector"] for x in hoop_positions])[:, :2]
    assert len(images) == len(ref_pts)

    img_pts = []
    img_circles = []
    for img, pt in zip(images, ref_pts):
        color_diffs = img - np.array([105, 75, 30]).reshape(1, 1, 3)
        color_dists = np.linalg.norm(color_diffs, axis=-1)
        mask = color_dists < 128
        mask = mask.astype(np.uint8) * 255

        circle = find_hoop_contour(mask)
        center = circle["center"]
        radius = circle["radius"]
        circle = np.array([center[0], center[1], radius])
        center = circle[:2]
        radius = circle[2]
        img_pts.append(center)
        img_circles.append(circle)

    img_pts = np.array(img_pts)

    H, _ = cv2.findHomography(img_pts, ref_pts)
    return H
