#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-4
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
# This example shows what transformation does with a vector and basic properties of
# transformation.
# Note, you need to complete robotics_toolbox/core/so2.py in order for script this to
# work.

from robotics_toolbox.core.so2 import SO2
from robotics_toolbox.render import RendererPlanar

# Let's create two frames, one reference/identity frame and one rotated by 45 degrees
t0 = SO2()  # identity
t1 = SO2(angle=45.0, degrees=True)

# Let's visualize them
renderer = RendererPlanar()
renderer.plot_so2(t0, length=0.1)
renderer.plot_so2(t1, length=0.2)
