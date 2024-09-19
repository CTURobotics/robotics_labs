# Installation

The robotics toolbox works with Python versions 3.10, 3.11 and 3.12. On Linux
it can be installed through conda or through a virtual environment and pip -
dependencies are managed through PDM.
On Windows and Mac, only conda is allowed as some of the dependencies are not available
through pip. See instructions below for each platform, we recommend using linux as it is
the most tested platform.

**Clone the repository.** Before installing, you need to first clone your repository (the copy you created from
the template) and navigate to the root of the repository.

```bash
git clone <your_github_repo>
cd <your_github_repo_name>
```

**CONDA** If you are going to use Conda (recommended for linux, required for Win/Mac), you need to have it installed. Other conda alternatives like miniconda are also fine.
For installation, see official [conda installation guide](https://docs.conda.io/projects/conda/en/latest/user-guide/install/index.html).
Our more detailed instructions for windows are available [here](./docs/conda_win_installation/README.md).

## Linux

Preferred option is through conda for reproducibility.

```bash
conda create -n ctu_robotics python=3.10
conda activate ctu_robotics
conda install -c conda-forge 'pdm>=2.15'
pdm install # this will install the toolbox and all the dependencies
```

If you dare, you can use PDM directly without conda:

```bash
# First install PDM https://pdm-project.org/en/latest/
pdm venv create -v --with-pip 3.10 
source .venv/bin/activate
pdm install
```

## Windows, MacOS, or Linux with dependencies installed through conda

```
conda create -n ctu_robotics python=3.10
conda activate ctu_robotics
conda install -c conda-forge pinocchio robomeshcat example-robot-data matplotlib numpy pycollada shapely anytree ruff black pytest
pip install --no-deps -e .
```
