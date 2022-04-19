cmake_minimum_required(VERSION 3.0.2)
project(sr_logging_common)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++17)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

file(GLOB SCRIPTS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/logging/*.py")
catkin_install_python(PROGRAMS ${SCRIPTS}
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
