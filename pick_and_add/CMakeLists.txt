cmake_minimum_required(VERSION 2.8.3)
project(pick_and_add)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  actionlib
  move_base_msgs
)




catkin_package(

)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(pick_and_add src/pick_and_add.cpp)


target_link_libraries(pick_and_add ${catkin_LIBRARIES})




