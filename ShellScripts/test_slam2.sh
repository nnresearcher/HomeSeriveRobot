#!/bin/sh
xterm -e "rosrun gmapping slam_gmapping" &
sleep 5
xterm  -e  " cd ~/catkin_ws; source devel/setup.bash; cd ~/catkin_ws/src/turtlebot_interactions/turtlebot_rviz_launchers/launch; roslaunch view_navigation.launch" &
sleep 5
xterm  -e  " cd ~/catkin_ws; source devel/setup.bash; cd ~/catkin_ws/src/turtlebot/turtlebot_teleop/launch; roslaunch keyboard_teleop.launch" 
