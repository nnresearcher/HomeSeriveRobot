#!/bin/sh
source devel/setup.bash

xterm  -e  " cd ~/catkin_ws; source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/robond/catkin_ws/src/maps/mymap.yaml" &

sleep 5
xterm  -e  " cd ~/catkin_ws; source devel/setup.bash; cd ~/catkin_ws/src/turtlebot_interactions/turtlebot_rviz_launchers/launch; roslaunch view_navigation.launch" &

sleep 5
xterm -e "cd ~/catkin_ws; source devel/setup.bash;rosrun add_markers add_markers" 
