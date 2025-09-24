cmake_minimum_required(VERSION 3.0.2)
project(topic_transformer)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

# No custom messages, services or actions needed

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs
  # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/udp_communicator.cpp
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(topic_transformer_node src/topic_transformer_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(topic_transformer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a executable target against
target_link_libraries(topic_transformer_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

#############
## Testing ##
#############

## Add gtest based cpp tests
if(CATKIN_ENABLE_TESTING)
  # find_package(rostest REQUIRED)
  # add_rostest_gtest(${PROJECT_NAME}_test test/test_topic_transformer.launch test/test_topic_transformer.cpp)
  # target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()