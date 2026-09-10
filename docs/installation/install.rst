========================
Installation instruction
========================

The robotics toolbox works with Python versions 3.10, 3.11 and 3.12 on Linux, Windows and
MacOS. All dependencies are installed through pip, no conda is needed.

Clone the repository
======================

Before installing, you need to first clone your repository (the copy you created from
the template) and navigate to the root of the repository.

.. code-block:: bash

    git clone <your_github_repo>
    cd <your_github_repo_name>

Creating environment and installing dependencies
================================================

Create a virtual environment with Python 3.10 - 3.12, activate it, and install the toolbox
together with its dependencies:

.. code-block:: bash

    python -m venv .venv
    source .venv/bin/activate         # Linux / MacOS
    .venv\Scripts\activate            # Windows (PowerShell or cmd)
    pip install -e .

If you use conda to manage Python versions, create the environment with conda and install
the toolbox with pip afterwards:

.. code-block:: bash

    conda create -n ctu_robotics python=3.10
    conda activate ctu_robotics
    pip install -e .

To verify the installation, run the tests of the provided utilities:

.. code-block:: bash

    pytest tests/test_geometry_utils.py

Robot models
------------

The models of the Panda, Talos and Tiago robots used in the exercises are downloaded
automatically from the internet by the `robot_descriptions` package the first time you use
them and cached in your home directory (`~/.cache/robot_descriptions`, several hundreds of
MB). Make sure you are online when running the spatial exercises for the first time.

Development installation
========================

The dependencies of the project are managed by `PDM <https://pdm-project.org/>`_, which
also provides scripts for testing, linting and formatting:

.. code-block:: bash

    pip install pdm
    pdm install      # installs the toolbox and dev dependencies into .venv
    pdm test
    pdm lint
    pdm format
