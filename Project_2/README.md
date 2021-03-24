# Project 2. Go Chase It!
Use ROS, C++, and Gazebo to build and program a ball-chasing robot. It consists of designing a robot inside Gazebo, house it in a world you have to build, 
and code a C++ node in ROS to chase a white ball.

The screenshot below shows RViz (left) and Gazebo (right). The robot that was created can be seen in green in Gazebo and in red in RViz. RViz also shows the readings of the robot's laser scanner. 

![Screenshot_RViz_and_Gazebo](Screenshot_RViz_and_Gazebo.png)

## To use
The folders `my_robot` and `ball_chaser` should be copied to your `catkin_ws/src` folder.

First, launch the simulation environment. Open a terminal and run:

```
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
In a new terminal, launch the ball_chaser:
```
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch ball_chaser ball_chaser.launch
```
Move the white ball in front of the robot. It should go after it.

## Requirements
To run the code from this repository you will need to use [ROS Kinetic](http://wiki.ros.org/kinetic).
