cmake_minimum_required(VERSION 3.0.2)
project(happy_walker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES happy_walker
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

 catkin_install_python(PROGRAMS
   scripts/talker.py scripts/image_sender.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
