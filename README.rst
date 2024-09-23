================
Robotics Toolbox
================

Robotics toolbox library is template repository that needs to be filled with content so all unit tests are passing.
It is used as template and verification tool for students laboratories of Robotics Course at Czech Technical University.


There are separate homework/labs assignments that guides you to implement the missing functionality step-by-step and verify it using independent unit tests.
There are two types of homework: (i) **mandatory**, which you need to finish to pass the course at CTU, and (ii) **optional**, for which you can get bonus points but which are not required to pass the course.
We internally use the same unit tests to verify that your implementation is correct.
For example, to test the first homework locally you can run:

.. code-block:: bash

    pytest tests/hw01 # you will get maximum points if all tests here pass
    pytest tests/hw01/mandatory # for mandatory part of the HW
    pytest tests/hw01/optional # for optional part of the HW

In addition to tests, there are exercises that use your implemented functionality and demonstrate the usage of the framework. See exercises folder.

You are allowed to use only the following software packages to implement missing parts of the code:
```python standard library, numpy, robomeshcat (only for visualization), anytree```


**BRUTE** To get points for the assignments in CTU evaluation system BRUTE, zip the src folder and upload the zip to the BRUTE system.
For example, to get points for HW01, upload the whole repository to BRUTE assignments hw01 before the deadline that you see in the system.

For more documentation see docs folder or generated documentation at https://robotics-labs.readthedocs.io/en/latest/ .
