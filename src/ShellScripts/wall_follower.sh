#!/bin/sh
source devel/setup.bash
xterm  -e  " cd ~/catkin_ws; source devel/setup.bash;cd ~/catkin_ws/src/turtlebot_simulator/turtlebot_gazebo/launch;roslaunch gmapping_demo2.launch " &
sleep 5
xterm  -e  " cd ~/catkin_ws; source devel/setup.bash; cd ~/catkin_ws/src/turtlebot_interactions/turtlebot_rviz_launchers/launch; roslaunch view_navigation.launch" &
sleep 5
xterm -e "cd ~/catkin_ws; source devel/setup.bash;rosrun wall_follower wall_follower" 
