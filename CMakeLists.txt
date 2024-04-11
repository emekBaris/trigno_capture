cmake_minimum_required(VERSION 3.0.2)
project(trigno_capture)
find_package(catkin REQUIRED COMPONENTS rospy)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  trignoIMU.msg
  trignoMultiIMU.msg
  trignoEMG.msg
  trignoMultiEMG.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  geometry_msgs
)
include_directories(include ${catkin_INCLUDE_DIRS})


