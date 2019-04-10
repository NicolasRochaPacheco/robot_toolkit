cmake_minimum_required(VERSION 2.8.3)
project(robot_toolkit)

set(CMAKE_AUTOMOC YES)
set(CMAKE_INCLUDE_CURRENT_DIR YES)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_toolkit
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

set(libs)

# Catkin
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
include_directories(${catkin_INCLUDE_DIRS})
list(APPEND libs ${catkin_LIBRARIES})

include_directories(${PROJECT_SOURCE_DIR})

set(SOURCES
    src/main.cpp
    src/robot_toolkit.cpp
)

set(HEADERS
   include/robot_toolkit/robot_toolkit.hpp
)

set(srcs
    ${SOURCES}
    ${HEADERS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node ${srcs})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${libs})

## Add cmake target dependencies of the executable
#add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )



