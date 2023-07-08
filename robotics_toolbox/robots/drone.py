#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-7
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from robotics_toolbox.core import SE3


class Drone:
    def __init__(self) -> None:
        super().__init__()
        self.pose = SE3()
