=================
Lab03: Perception
=================

In this lab we will show you how to use OpenCV for various vision tasks.
First, download the required lab data at
https://data.ciirc.cvut.cz/public/projects/2025CTURoboticsData/perception_hw_data.zip
and unzip it to the `excercises/lab03` directory. Now you should have there `hw_data`
directory with all the files required for the homework.

Tests
=====

Mandatory HW:
 - `src/robotics_toolbox/utils/perception.py`
     - implement the `find_hoop_homography` function:
       Given `N` images as NumPy arrays and `N` SE(3) poses of the hoop on a plane `P`,
       find a homography that transforms points **from the image plane to the plane P**.
       First, detect the hoop in the images (you can use e.g. thresholding, followed by
       circle detection using Hough transform, or extract contours and measure their "roundness").
       Second, use the detected hoop/circle centers and provided poses to compute the homography
       `H` using `cv2.findHomography` function.

If following tests will pass, your implementation is correct:

.. code-block:: bash

    pytest tests/hw03/mandatory

After you implement the function, you should also be able to visualize the circle centers
by running the `excercises/lab03/03_homography.py` script, which loads the
provided points (hoop poses) and transforms them back to image plane using inverse of H.

At the end of the lab, you should be able to answer following questions:

- how to detect ArUco markers using OpenCV
- what are homogeneous coordinates
- what is homography matrix and how many degrees of freedom does it have
- how to find homography using OpenCV