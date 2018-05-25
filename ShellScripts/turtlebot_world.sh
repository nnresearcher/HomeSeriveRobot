#!/bin/sh
xterm -e "cd ~/catkin_ws; source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/robond/catkin_ws/src/World/home_service.world" 

