#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-09-19
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

from pathlib import Path
from shutil import rmtree
from subprocess import call

from imageio import imwrite
from matplotlib import pyplot as plt
from robotics_toolbox.render import RendererSpatial, RendererPlanar


def save_fig(output_folder: Path | str, renderer=None):
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
        if isinstance(renderer, RendererSpatial):
            imwrite(img_path, renderer.render_image())
        elif isinstance(renderer, RendererPlanar):
            renderer.fig.savefig(img_path)
    else:
        plt.savefig(img_path)
    save_fig.last_fig_id[output_folder] += 1


def create_mp4_from_folder(
    folder: Path | str, output: Path | str | None = None, fps: int = 10
):
    folder = Path(folder)
    if output is None:
        output = folder.joinpath("animation.mp4")
    output = Path(output)
    assert output.suffix == ".mp4"

    call(
        f"ffmpeg -y -framerate {fps} -i {folder}/img_%00d.png -c:v libx264"
        f" -profile:v high -crf 20 -pix_fmt yuv420p {output}",
        shell=True,
    )
    return output


def create_gif_from_mp4(input: Path | str, output: Path | str | None = None):
    input = Path(input)
    if output is None:
        output = input.parent.joinpath("animation.gif")
    output = Path(output)
    assert output.suffix == ".gif"
    output_palette = output.parent.joinpath(f"{output.stem}_palette.png")

    call(f"ffmpeg -y -i {input} -vf palettegen {output_palette}", shell=True)
    call(
        f"ffmpeg -y -i {input} -i {output_palette} -filter_complex paletteuse -r 10 {output}",
        shell=True,
    )
