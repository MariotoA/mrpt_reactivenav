cmake_minimum_required(VERSION 3.1)
ADD_COMPILE_OPTIONS("-std=c++17")
project(mrpt_reactivenav CXX)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})
message(STATUS "Flags: " ${CMAKE_CXX_FLAGS})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  dynamic_reconfigure
  geometry_msgs
  mrpt_bridge
  roscpp
  tf
  visualization_msgs
  pluginlib
  nav_core
  shape_publisher
  pcl_ros
  laser_geometry

)

## System dependencies are found with CMake's conventions
# find_package(MRPT REQUIRED base reactivenav kinematics)
find_package(MRPT REQUIRED nav kinematics obs maps gui)
MESSAGE( STATUS "MRPT_VERSION: ${MRPT_VERSION}")
MESSAGE( STATUS "MRPT_INCLUDE_DIRS: ${MRPT_INCLUDE_DIRS}")

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES test_lib #${PROJECT_NAME}
  CATKIN_DEPENDS actionlib actionlib_msgs dynamic_reconfigure geometry_msgs mrpt_bridge roscpp tf visualization_msgs pluginlib nav_core
  #DEPENDS mrpt
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${MRPT_INCLUDE_LIBS}
)
add_library(test_lib src/testlib.cpp)
add_dependencies(test_lib ${catkin_EXPORTED_TARGETS} ${MPRT_EXPORTED_TARGETS})
target_link_libraries(test_lib ${INCLUDE} ${MRPT_LIBRARIES} ${catkin_LIBRARIES})


add_executable(mrpt_reactivenav_node src/mrpt_reactivenav_node.cpp)
target_link_libraries(mrpt_reactivenav_node
  ${catkin_LIBRARIES}
  ${MRPT_LIBS}
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(mrpt_reactivenav2d_node mrpt_reactivenav2d_generate_messages_cpp)

## Specify libraries to link a library or executable target against


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html


## Mark executables and/or libraries for installation mrpt_reactivenav_node
install(TARGETS test_lib #${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)

install(DIRECTORY
  launch
  tutorial
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "test_lib/*.h"
  )


install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
