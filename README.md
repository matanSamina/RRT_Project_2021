# Voronoi + A* + RRT

This repository is a part of an assignment in
Robot's Control and Navigation Course (2021 Semester B).

## Paper

[Optimal Path Planning Using Generalized Voronoi Graph and Multiple Potential Functions](https://ieeexplore.ieee.org/document/8948325)

## The Structure

The main parts of code are located in [/src](/src) directory.

[`a_star.py`](/src/astar.py) - our simple and generic implementation of A*.
More explanation about the A* implementation can be found
in [this](https://github.com/Arseni1919/A_star_Implementation) repository.

[`RRT.py`](/src/astar.py) - our simple and generic implementation of RRT.
More explanation about the RRT implementation can be also found
in [this](https://github.com/Arseni1919/Simple_Implementation_of_RRT) repository.

[`main_find_path.py`](/src/main_find_path.py) - the main flow of path planning.
The file contains ROS node that responsible to build a path for robot.

## Simulation

We worked with TurtleBot3 robot, Gazebo simulation and Rviz.

TurtleBot3 | Rviz
------------ | -------------
![](static/Picture1.png) | ![](static/Picture4.png)

Gazebo (1) | Gazebo (2)
------------ | -------------
![](static/Picture2.png) | ![](static/Picture3.png)

## The Main Logic of The Algorithm

### Find

## Credits

- [ROS - `tuw_voronoi_graph`](http://wiki.ros.org/tuw_voronoi_graph)
- [ROS - Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)
- [ROS - Point](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Point.html)




