#!/bin/sh

# Go to the directory where catkin_ws is located (not necessary if you run the script from the catkin_ws directory).
cd /home/workspace/catkin_ws

export TURTLEBOT_GAZEBO_WORLD_FILE="$(pwd)/src/worlds/myworld_project_no_wall.world"

# Testing
#xterm -e " cd $(pwd) && source devel/setup.bash && pwd && roscore"
#sleep 1

xterm -e " cd $(pwd) && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 15

#xterm -e " cd $(pwd) && source devel/setup.bash && roslaunch turtlebot_gazebo gmapping_demo.launch" &
xterm -e " cd $(pwd) && source devel/setup.bash && roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/src/map/map_slam.yaml" &
sleep 10

xterm -e " source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 7

xterm -e " source devel/setup.bash && roslaunch turtlebot_teleop keyboard_teleop.launch"
