#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2025-09-21
#     Author: Martin Cífka <martin.cifka@cvut.cz>
#
import unittest
import numpy as np
import cv2
from pathlib import Path
from robotics_toolbox.utils.perception import find_hoop_homography
import json


class TestHomograpy(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestHomograpy, self).__init__(*args, **kwargs)
        repo_dir = Path(__file__).parent.parent.parent.parent
        data_dir = repo_dir / "exercises" / "lab03" / "hw_data"

        images_paths = [str(data_dir / f"hoop_grid_image_{i}.png") for i in range(17)]
        self.images = [cv2.imread(pth) for pth in images_paths]

        with open(data_dir / "hoop_positions.json") as f:
            self.hoop_positions = json.load(f)

    def test_homography_pts(self):
        H = find_hoop_homography(self.images, self.hoop_positions)

        img_pts = np.array(
            [
                [5.00000e-01, 5.00000e-01, 1.0],
                [5.00000e-01, 4.80250e02, 1.0],
                [5.00000e-01, 9.60000e02, 1.0],
                [5.00000e-01, 1.43975e03, 1.0],
                [5.00000e-01, 1.91950e03, 1.0],
                [1.50000e00, 5.00000e-01, 1.0],
                [1.50000e00, 4.80250e02, 1.0],
                [1.50000e00, 9.60000e02, 1.0],
                [1.50000e00, 1.43975e03, 1.0],
                [1.50000e00, 1.91950e03, 1.0],
                [2.50000e00, 5.00000e-01, 1.0],
                [2.50000e00, 4.80250e02, 1.0],
                [2.50000e00, 9.60000e02, 1.0],
                [2.50000e00, 1.43975e03, 1.0],
                [2.50000e00, 1.91950e03, 1.0],
                [3.50000e00, 5.00000e-01, 1.0],
                [3.50000e00, 4.80250e02, 1.0],
                [3.50000e00, 9.60000e02, 1.0],
                [3.50000e00, 1.43975e03, 1.0],
                [3.50000e00, 1.91950e03, 1.0],
                [4.50000e00, 5.00000e-01, 1.0],
                [4.50000e00, 4.80250e02, 1.0],
                [4.50000e00, 9.60000e02, 1.0],
                [4.50000e00, 1.43975e03, 1.0],
                [4.50000e00, 1.91950e03, 1.0],
            ]
        )

        est_plane_pts = img_pts @ H.T
        est_plane_pts /= est_plane_pts[:, 2].reshape(-1, 1)

        ref_plane_pts = np.array(
            [
                [0.29724235, -0.23586451, 1.0],
                [0.41216197, -0.2368604, 1.0],
                [0.52826821, -0.23786658, 1.0],
                [0.64557956, -0.2388832, 1.0],
                [0.76411486, -0.23991043, 1.0],
                [0.29724287, -0.23563076, 1.0],
                [0.41216249, -0.23662546, 1.0],
                [0.52826875, -0.23763042, 1.0],
                [0.6455801, -0.23864582, 1.0],
                [0.76411542, -0.23967181, 1.0],
                [0.29724338, -0.23539701, 1.0],
                [0.41216302, -0.23639051, 1.0],
                [0.52826928, -0.23739426, 1.0],
                [0.64558065, -0.23840843, 1.0],
                [0.76411597, -0.23943318, 1.0],
                [0.29724389, -0.23516326, 1.0],
                [0.41216354, -0.23615556, 1.0],
                [0.52826981, -0.2371581, 1.0],
                [0.64558119, -0.23817104, 1.0],
                [0.76411653, -0.23919456, 1.0],
                [0.2972444, -0.23492952, 1.0],
                [0.41216406, -0.23592061, 1.0],
                [0.52827035, -0.23692194, 1.0],
                [0.64558174, -0.23793365, 1.0],
                [0.76411708, -0.23895593, 1.0],
            ]
        )

        self.assertTrue(np.allclose(est_plane_pts, ref_plane_pts, rtol=5e-3, atol=5e-3))


if __name__ == "__main__":
    unittest.main()
