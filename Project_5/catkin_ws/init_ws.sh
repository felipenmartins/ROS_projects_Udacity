#!/bin/sh

pip install rospkg
cd /home/workspace/catkin_ws
catkin_make
source devel/setup.bash

