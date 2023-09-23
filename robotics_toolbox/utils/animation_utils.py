#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-19
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

from __future__ import annotations
from pathlib import Path
from shutil import rmtree
from subprocess import call

from imageio import imwrite
from matplotlib import pyplot as plt


def save_fig(output_folder: Path | str = "/tmp/animation", renderer=None):
    """Save fig of the renderer into the given output folder. Output folder is cleaned
    on the first run of this command. If renderer not provided use plt.savefig.
    This name of figures is img_{id}.png."""

    if not hasattr(save_fig, "last_fig_id"):
        save_fig.last_fig_id = {}

    output_folder = Path(output_folder)
    if output_folder not in save_fig.last_fig_id:
        save_fig.last_fig_id[output_folder] = 0
        if output_folder.exists():
            rmtree(output_folder)
        output_folder.mkdir(parents=True)

    img_path = output_folder.joinpath(f"img_{save_fig.last_fig_id[output_folder]}.png")
    if renderer is not None:
        from robotics_toolbox.render import RendererSpatial, RendererPlanar

        if isinstance(renderer, RendererSpatial):
            imwrite(img_path, renderer.render_image())
        elif isinstance(renderer, RendererPlanar):
            renderer.fig.savefig(img_path)
    else:
        plt.savefig(img_path)
    save_fig.last_fig_id[output_folder] += 1


def create_mp4_from_folder(
    folder: Path | str = "/tmp/animation",
    output: Path | str | None = None,
    fps: int = 10,
):
    """From the folder that contains images names img_X.png, create mp4 animation."""
    folder = Path(folder)
    if output is None:
        output = folder.joinpath("animation.mp4")
    output = Path(output)
    assert output.suffix == ".mp4"

    call(
        f"ffmpeg -y -framerate {fps} -i {folder}/img_%00d.png -c:v libx264"
        f" -profile:v high -crf 20 -pix_fmt yuv420p"
        ' -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2"'
        f" {output}",
        shell=True,
    )
    return output


def create_gif_from_mp4(input_vid: Path | str, output: Path | str | None = None):
    """Convert input_vid (mp4) to GIF by first generating the color pallet."""
    input_vid = Path(input_vid)
    if output is None:
        output = input_vid.parent.joinpath("animation.gif")
    output = Path(output)
    assert output.suffix == ".gif"
    output_palette = output.parent.joinpath(f"{output.stem}_palette.png")

    call(f"ffmpeg -y -i {input_vid} -vf palettegen {output_palette}", shell=True)
    call(
        f"ffmpeg -y -i {input_vid} -i {output_palette} -filter_complex paletteuse "
        f"-r 10 {output}",
        shell=True,
    )
