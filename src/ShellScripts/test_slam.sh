#!/bin/sh
xterm  -e  " cd ~/catkin_ws; source devel/setup.bash; cd ~/catkin_ws/src/turtlebot_simulator/turtlebot_gazebo/launch; roslaunch turtlebot_world.launch" &
sleep 20
xterm  -e  " cd ~/catkin_ws; source devel/setup.bash; cd ~/catkin_ws/src/slam_gmapping/gmapping/launch; roslaunch slam_gmapping_pr2.launch" &
sleep 5
xterm  -e  " cd ~/catkin_ws; source devel/setup.bash; cd ~/catkin_ws/src/turtlebot_interactions/turtlebot_rviz_launchers/launch; roslaunch view_navigation.launch" &
sleep 5
xterm  -e  " cd ~/catkin_ws; source devel/setup.bash; cd ~/catkin_ws/src/turtlebot/turtlebot_teleop/launch; roslaunch keyboard_teleop.launch" 
