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
        data_dir = repo_dir / "exercises" / "lab05_perception" / "hw_data"

        images_paths = [str(data_dir / f"hoop_grid_image_{i}.png") for i in range(17)]
        self.images = [cv2.imread(pth) for pth in images_paths]
        
        with open(data_dir / 'hoop_positions.json') as f:
            self.hoop_positions = json.load(f)
        
        self.H_true = np.array([
            [ 4.91643554e-07,  2.35149198e-04,  2.97122940e-01],
            [ 2.33763483e-04,  4.47069090e-07, -2.35980350e-01],
            [-6.95833026e-08, -1.06515072e-05,  1.00000000e+00]])

    def test_homography_eq(self):
        H = find_hoop_homography(self.images, self.hoop_positions)
        self.assertTrue(np.allclose(H, self.H_true, rtol=5e-3, atol=5e-3))

if __name__ == "__main__":
    unittest.main()
