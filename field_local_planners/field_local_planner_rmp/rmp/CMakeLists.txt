cmake_minimum_required(VERSION 2.8.3)
project(rmp)
set (CMAKE_CXX_STANDARD 14)

set(CATKIN_PACKAGE_DEPENDENCIES
  gtsam_catkin
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWitDebInfo)
endif()

find_package(Eigen3 REQUIRED)

###################################
## Catkin Specific Configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS 
)

###########
## Build ##
###########
include_directories(
  include
  test
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

######################################################
add_library(${PROJECT_NAME} SHARED
  src/models/ControlPoint.cpp
  src/models/AckermannModel.cpp
  src/models/DifferentialModel.cpp
  src/rmp.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES})


##########
## Test ##
##########
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_test test/main.cpp
                                        test/unitTests.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
endif()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})