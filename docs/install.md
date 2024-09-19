# Installation

The robotics toolbox works with Python versions 3.10, 3.11 and 3.12. On Linux
it can be installed through conda or through a virtual environment and pip -
dependencies are managed through PDM.
On Windows and Mac, only conda is allowed as some of the dependencies are not available
through pip. See instructions below for each platform, we recommend using linux as it is
the most tested platform.

## Linux

Preferred option is through conda for reproducibility.
todo: conda installation

## Virtual Environment through PDM

If you dare, you can use PDM directly without conda:

```bash
# First install PDM https://pdm-project.org/en/latest/
pdm venv create -v --with-pip 3.10 
source .venv/bin/activate
pdm install
```

# Windows

```
pip install --no-deps -e .
```