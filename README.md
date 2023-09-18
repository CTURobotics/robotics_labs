# CTU Robotics Course - Code for Laboratories 
This repository contains code base for creating a robotic toolbox in python.
It is a template code that you need to complete in order to be useful.
Our homework will guide you to complete it successfully.

There are two types of homework: (i) mandatory, that you need to finish to pass the course at CTU, and (ii) voluntary, for which you can get bonus points but are not required to pass the course.
We use unit tests to verify that your implementation is correct.
For example, to test the first homework locally you can run:

```bash
pytest tests/hw01 # you will get maximum points if all tests here passes
pytest tests/hw01/mandatory # for mandatory part of the HW
pytest tests/hw01/optional # for optional part of the HW
```
In addition to tests, there are exercises that use your implemented functionality and demonstrates the usage of the framework. See [exercises](exercises) folder.

You are allowed to use only following software packages to implement missing parts of the code:
```
python standard library, numpy, robomeshcat (only for visualization) 
```


## Installation
The best way to start developing your own version of the toolbox, is to use this template repository to create your own **private** repository on GitHub.

**_TBD:_** create a gif instruction how to do that

We recommend to use conda to create replicable environment:
```bash
conda create -n ctu_robotics python=3.10 poetry
conda activate ctu_robotics
# Clone your repository and install it
git clone <your_github_repo>
poetry install # this will install the toolbox
```

# Homework and laboratories
- [Lab01](docs/lab01.md) - transformations
