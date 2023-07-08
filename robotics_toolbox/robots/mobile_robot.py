#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-5
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from robotics_toolbox.core import SE2


class MobileRobot:
    def __init__(self, size: float = 0.3) -> None:
        super().__init__()
        self.pose = SE2()
        self.size = size
