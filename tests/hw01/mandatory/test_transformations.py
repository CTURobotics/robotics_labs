#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-4
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import unittest

import numpy as np

from robotics_toolbox.core.so2 import SO2


class TestTransformations(unittest.TestCase):
    def test_so2_initialization(self):
        self.assertEqual(SO2(np.pi / 4), SO2(45.0, degrees=True))
# todo: test there is only rot variable inside the class
# todo: test some of the rotation matrices - use pinocchio for creating reference


if __name__ == "__main__":
    unittest.main()
