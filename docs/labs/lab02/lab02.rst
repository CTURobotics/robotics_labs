==========================
Lab02: Forward kinematics
==========================

The goal of this laboratory is twofold: (i) implement forward kinematics of the planar manipulator and (ii) create-and-animate urdf file that describes spatial robot.

Forward kinematics of planar manipulator
========================================

Your goal is to implement forward kinematics for all links for a planar manipulator that is composed of revolute and prismatic joints.
The links frames are defined as follows:

- the first frame is located in the base_pose of the robot,
- for links attached to parent by revolute joints, the frame origin is at the end of the link with x-axis pointing in the direction of the link,
- for links attached to parent by prismatic joints, the x-axis points in the direction of motion and the origin is located at the end of the link.

The visualization of the frames created by `exercises/lab02_forward_kinematics/animate_robot_with_all_frames.py`:

.. image:: lab02_fk_animation.gif
    :width: 800px
    :align: center


URDF
====

Your goal is to create robot description in an URDF file `src/robotics_toolbox/robots/robot_hw.urdf` with a description of spatial robot with 4 DoF.
The robot is made of the following links and joints:

- a cube base,
- revolute joint around Z axis between the cube base and the first cylinder link,
- first cylinder link placed atop the cuboid base,
- first sphere link placed atop the first cylinder link (connected to previous link by fixed joint),
- revolute joint around X axis between the first sphere link and the following second cylinder link,
- second cylinder link connected to the first sphere via aforementioned joint,
- revolute joint around Z axis between the second cylinder link and the following third cylinder link,
- third cylinder link placed atop the second cylinder link,
- second sphere link placed atop the third cylinder link (connected to previous link by fixed joint),
- revolute joint around X axis between the second sphere link and the following fourth cylinder link,
- fourth cylinder link connected to the second sphere via aforementioned joint,
- fictional end effector link without geometry placed at the top of the fourth cylinder (connected to the cylinder by fixed joint),

All joints have the same limits: -pi/2 to pi/2.



See the drawings bellow for the dimensions and animation of the expected structure.

.. image:: lab02_spatial.gif
    :width: 800px
    :align: center

Description of the robot dimensions in millimeters:

.. image:: lab02_spatial_description.png
    :width: 800px
    :align: center

Positions of the frames of the links of the robot:

.. image:: lab02_spatial_frames.png
    :width: 800px
    :align: center


Tests
=====

Mandatory HW:
 - `robotics_toolbox/robots/planar_manipualator.py`
   - implement `fk_all_links` and `flange_pose` functions
 - `src/robotics_toolbox/robots/robot_hw.urdf`
   - create an URDF and store it in `exercises/lab02/` folder

If following tests will pass, your implementation is correct:

.. code-block:: bash

    pytest tests/hw02/mandatory
