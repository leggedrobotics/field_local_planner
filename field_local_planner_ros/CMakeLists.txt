cmake_minimum_required(VERSION 3.0.2)
project(field_local_planner_ros)
set (CMAKE_CXX_STANDARD 14)

set(DEFAULT_BUILD "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
   message(STATUS "Setting build type to '${DEFAULT_BUILD}' as none was specified.")
   set(CMAKE_BUILD_TYPE ${DEFAULT_BUILD} CACHE STRING "Choose the type of build." FORCE)
   # Set the possible values of build type for cmake-gui
   set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug"
                                                        "Release"
                                                        "MinSizeRel"
                                                        "RelWithDebInfo")
endif()

set(CATKIN_PACKAGE_LIST 
  field_local_planner_base_plugin
  pluginlib
  roscpp
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_LIST}
)

# Libraries dependencies
# configure catkin package
catkin_package(
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_LIST}
)

###########
## Build ##
###########
include_directories(
  SYSTEM
    ${catkin_INCLUDE_DIRS}
)

# Declare node
add_executable(field_local_planner_node
               src/local_planner_node.cpp)

target_link_libraries(field_local_planner_node
                      ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

############
## Python ##
############
catkin_install_python(PROGRAMS
    scripts/action_client.py
    scripts/action_client_posestamped.py
    scripts/carrot_publisher.py
    scripts/move_base_bridge.py
    scripts/move_base_simple.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
