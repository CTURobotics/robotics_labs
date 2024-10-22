#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2024-10-21
#     Author: Martin Cífka <martin.cifka@cvut.cz>
#
import cv2
import numpy as np
from perception_utils import viz_calibration
from argparse import ArgumentParser
from pathlib import Path
from perception_utils import max_resize
from matplotlib import pylab as plt


def calibrate(dir, grid, square_size, img_max_width=1024, viz=False):
    extensions = (".jpg", ".jpeg", ".png")
    images = Path(dir).iterdir()
    images = sorted([x for x in images if str(x).lower().endswith(extensions)])[:7]

    # Define the 3D points of the chessboard corners (x,y,0)
    corners3d = np.zeros((1, grid[0] * grid[1], 3), np.float32)
    corners3d[0, :, :2] = np.mgrid[0 : grid[0], 0 : grid[1]].T.reshape(-1, 2)
    corners3d *= square_size  # needed for proper extrinsics only

    # Get the 2D points of the chessboard corners in each image
    pts2d = []
    camera_idxs = []
    for i, fname in enumerate(images):
        # Read and preprocess the image
        img = cv2.imread(fname)
        img = max_resize(img, max_width=img_max_width)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if i == 0:
            h, w = img.shape[:2]
            print(f"Using images preprocessed to size {w}x{h}")
        else:
            assert img.shape[:2] == (h, w), "Inconsistent image dimensions"

        # Try to find the chessboard corners
        criteria = (
            cv2.CALIB_CB_ADAPTIVE_THRESH
            + cv2.CALIB_CB_FAST_CHECK
            + cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        success, corners = cv2.findChessboardCorners(gray, grid, criteria)

        if success:
            camera_idxs.append(i)

            # Subpixel refinement of the corners
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            pts2d.append(corners)

            if viz:
                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, grid, corners, success)

        if viz:
            fig, ax = plt.subplots(figsize=(10, 7))
            ax.imshow(img[:, :, ::-1])
            plt.tight_layout()
            plt.show()

    pts3d = [corners3d for _ in range(len(pts2d))]
    err, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        pts3d, pts2d, gray.shape[::-1], None, None
    )
    np.savez(
        "calibration.npz",
        K=K,
        dist=dist,
        rvecs=rvecs,
        tvecs=tvecs,
        max_width=img_max_width,
    )

    print(
        f"RMS re-projection error: {err}\n",
    )
    print(f"Camera matrix:\n{K}\n")
    print(f"Distortion coefficients:\n{dist}\n")

    if viz:
        viz_calibration(
            corners3d,
            grid,
            rvecs,
            tvecs,
            camera_idxs=camera_idxs,
            axis_length=2.2 * square_size,
            text_size=15,
        )

    return err, K, dist, rvecs, tvecs


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-d", default="chessboard", type=str, help="Dir with images")
    parser.add_argument("-w", type=int, default=1024, help="Max image width")
    parser.add_argument("-g", type=str, default="6x9", help="Grid size")
    parser.add_argument("-s", type=float, default=30.0, help="Square size")

    args = parser.parse_args()

    # Get the chessboard grid size
    grid = tuple(map(int, args.grid.split("x")))
    assert len(grid) == 2, "Invalid grid size"

    calibrate(
        args.dir, grid, args.square_size, img_max_width=args.width, viz=not args.no_show
    )
