cmake_minimum_required(VERSION 3.0.2)
project(sr_controllers_tools)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
   src/sr_controllers_tools/sr_controller_helper.py
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
