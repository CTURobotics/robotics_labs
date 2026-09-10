========================
Installation instruction
========================

The robotics toolbox works with Python versions 3.10, 3.11 and 3.12 on Linux, Windows and
MacOS. Python, the virtual environment and all dependencies are managed by
`uv <https://docs.astral.sh/uv/>`_, no conda is needed.

Install uv
==========

Follow the `official uv installation guide
<https://docs.astral.sh/uv/getting-started/installation/>`_, e.g.:

.. code-block:: bash

    curl -LsSf https://astral.sh/uv/install.sh | sh                                          # Linux / MacOS
    powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"      # Windows

or ``pip install uv`` if you already have some Python installed.

Clone the repository
====================

Before installing, you need to first clone your repository (the copy you created from
the template) and navigate to the root of the repository.

.. code-block:: bash

    git clone <your_github_repo>
    cd <your_github_repo_name>

Creating environment and installing dependencies
================================================

A single command creates the virtual environment ``.venv`` with Python 3.12 (downloaded
automatically if it is not installed) and installs the toolbox with all dependencies:

.. code-block:: bash

    uv sync

To run python or the tests inside the environment, either prefix the commands with
``uv run``:

.. code-block:: bash

    uv run pytest tests/test_geometry_utils.py   # verify the installation
    uv run pytest tests/hw01                     # test your homework
    uv run python exercises/lab01/01_so2_example.py

or activate the environment once and use the commands directly:

.. code-block:: bash

    source .venv/bin/activate         # Linux / MacOS
    .venv\Scripts\activate            # Windows (PowerShell or cmd)
    pytest tests/hw01

Installation without uv
-----------------------

If you prefer plain pip (or an environment created by conda), create a Python 3.10 - 3.12
environment, activate it and install the toolbox from the repository root:

.. code-block:: bash

    python -m venv .venv
    source .venv/bin/activate         # Linux / MacOS
    .venv\Scripts\activate            # Windows (PowerShell or cmd)
    pip install -e .

Robot models
------------

The models of the Panda, Talos and Tiago robots used in the exercises are downloaded
automatically from the internet by the `robot_descriptions` package the first time you use
them and cached in your home directory (`~/.cache/robot_descriptions`, several hundreds of
MB). Make sure you are online when running the spatial exercises for the first time.

Development
===========

Development dependencies (pytest, ruff, black, sphinx) are installed by ``uv sync`` as
well. Useful commands:

.. code-block:: bash

    uv run pytest tests/                         # run all the tests
    uv run ruff check src tests exercises        # lint
    uv run black src tests exercises             # format
    uv run sphinx-build -b html docs/ docs/_build   # build the documentation
    uv lock --upgrade                            # update the dependency lock file
