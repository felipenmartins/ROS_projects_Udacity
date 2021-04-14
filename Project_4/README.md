# Project 4. Map My World
Interface your robot with an RTAB Map ROS package to localize it and build 2D and 3D maps of their environment. 

## Generated db files
The generated db files are in the folder '/catkin_ws/src/my_robot/db_files'.

## To use
To map the environment, first, launch the Gazebo world and RViz, spawn the robot in the environment. Open a terminal and run:

```
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch my_robot world_with_robot.launch
```

In a new terminal, run the teleop node to move the robot around:
```
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

In a third terminal, launch the mapping.launch file to run the RTAB-Map SLAM:
```
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch my_robot mapping.launch
```

Now, select the teleop terminal and use the keys to navigate the robot to create a map of the whole environment. After terminating the node, find the map db file in the /root/.ros/ folder.

## Requirements
To run the code from this repository you will need to use [ROS Kinetic](http://wiki.ros.org/kinetic).
