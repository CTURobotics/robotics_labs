# Lab06 - Motion planning

The goal is to implement RRT algorithm for path planning and random shortcut algorithm for path simplification.
If planner works, you can visualize paths for various motion planning problems with scripts in `exercises/lab06` folder, e.g.:

![](lab06_mobile_robot.gif)

![](lab06_planar.gif)

![](lab06_spatial.gif)

Path simplification visualization:

![](lab06_planar_simplified.gif)


## Tests

Optional HW:

- `robotics_toolbox/planning/rrt.py`
    - implement `plan` function
    - implement `random_shortcut` function

If following tests will pass, your implementation is correct:

```bash
pytest tests/hw06/optional
```
