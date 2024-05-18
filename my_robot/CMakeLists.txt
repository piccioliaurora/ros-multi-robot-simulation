cmake_minimum_required(VERSION 3.0.2)
project(my_robot)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_robot
#  CATKIN_DEPENDS gazebo_ros rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

