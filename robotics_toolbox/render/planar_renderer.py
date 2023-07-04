#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-4
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

import matplotlib.pyplot as plt

from robotics_toolbox.core import SO2


class RendererPlanar:
    def __init__(self) -> None:
        super().__init__()
        self.fig, self.ax = plt.subplots(
            1, 1, squeeze=True
        )  # type: plt.Figure, plt.Axes
        plt.ion()

    def plot_so2(self, t: SO2, length=0.1):
        """Plot SO2 frame in the origin."""
        pass

    def plot_se2(self, t, length=0.1):
        pass
