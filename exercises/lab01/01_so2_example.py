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
import numpy as np

from robotics_toolbox.core.so2 import SO2
from robotics_toolbox.render import RendererPlanar


# Let's visualize a few frames
renderer = RendererPlanar(xlim=(-0.25, 0.4), ylim=(-0.25, 0.4))

t0 = SO2()  # identity
renderer.plot_so2(t0, lw=1)
# renderer.wait_for_enter()

t1 = SO2(angle=np.deg2rad(45))
renderer.plot_so2(t1, lw=2)
# renderer.wait_for_enter()

# Let's have a look at the transformation of the vector

v_0 = [0.3, 0.2]
renderer.plot_line_between_points([0, 0], v_0, color="black")
# renderer.wait_for_enter()

# What is v_1, i.e. same point but different coordinate system?
print(t1.act(v_0))
# or it should be?
print(t1.inverse().act(v_0))
renderer.wait_for_enter()

v_1 = t1.inverse().act(v_0)
# Why we cannot use values of v_1 for plotting? ...

# If we have vectors expressed in coordinate system A, we need to transfer it to our
# reference coordinate frame to plot. We can do it by transformation from A to reference
v_plot = t1.act(v_1)
renderer.plot_line_between_points([0, 0], v_plot, "--y", lw=2)
renderer.wait_for_enter()

# Composition of rotation
t2 = SO2(angle=np.deg2rad(60))
renderer.plot_so2(t2, lw=3)
renderer.wait_for_enter()

t_composed = t1 * t2
renderer.plot_so2(t_composed, lw=4)
renderer.wait_for_close()
