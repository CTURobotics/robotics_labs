=====================
Lab01: Transformation
=====================

The goal of the first laboratories is to create a set o tools for transformations between frames both in 2D and 3D.
The core functionality needs to be implemented in `robotics_toolbox/core/{so2,se2,so3,se3}.py` files.
The parts to be implemented are marked with `TODO` comments.

Homework
========

Homework is split into:
- mandatory part, that you need to finish in order to implement functionalities in the following weeks and to pass the course, and
- optional part, for which you can get optional points but the functionality itself is not needed.

**Mandatory HW:**

- `robotics_toolbox/core/so2.py`
    - rotation matrix from a given angle
    - rotation composition via `__mul__`
    - angle computation from rotation matrix
    - inverse computation
- `robotics_toolbox/core/se2.py`
    - composition of transformations
    - inverse
    - act on a vector
- `robotics_toolbox/core/so3.py`
    - exp mapping, i.e. Rodrigues' formula
    - log mapping
    - composition of rotations
    - inverse
- `robotics_toolbox/core/se3.py`
    - composition of transformations
    - inverse
    - act on a vector

**Optional HW:**

- `robotics_toolbox/core/so3.py`
    - compute rotation that rotates about {x,y,z} axis by a given angle
    - compute from/to quaternion
    - compute from/to angle-axis
    - compute from euler-angles

Testing the implementation
--------------------------

If following tests will pass, your implementation is correct:

.. code-block:: bash

    pytest tests/hw01/mandatory
    pytest tests/hw01/optional

To understand transformations better, there are additional scripts in _excersies_ folder.
The scripts animate mobile robot with help of SE2 and drone with help of SE3.
To play the scripts, you need to have the implementation completed.

Please check the template repository for the structure of the repository and the expected code style.
At the end of the lab, please upload your solution to the Brute system to verify the correctness of your partial implementation.
You should be able to answer following questions at the end of the homework:

- how to make own repository from the GitHub template or how to download it as a zip
- what is unittest
- what is @property
- what is `__mul__`
- what is `__eq__`
- how to run the tests
- how to upload to brute, what is the deadline, and how brute evaluates the homework
