# Lab02 - Forward kinematics

The goal of this laboratory is twofold: (i) implement forward kinematics of the planar manipulator and (ii) create-and-animate urdf file that describes spatial robot.

## Forward kinematics of planar manipulator

Your goal is to implement forward kinematics for all links for a planar manipulator that is composed of revolute and prismatic joints.
The links frames are defined as follows:
 - the first frame is located in the base_pose of the robot,
 - for links attached to parent by revolute joints, the frame origin is at the end of the link with x-axis pointing in the direction of the link,
 - for links attached to parent by prismatic joints, the x-axis points in the direction of motion and the origin is located at the end of the link.

 The visualization of the frames created by `lab02/animate_robot_with_all_frames.py`:
 ![](lab02_fk_animation.gif)


## URDF

TBD: describes the kinematics of the spatial robot to implement

## Tests

Mandatory HW:
 - `robotics_toolbox/robots/planar_manipualator.py`
   - implement `fk_all_links` and `flange_pose` functions

If following tests will pass, your implementation is correct:
```bash
pytest tests/hw01/mandatory
```