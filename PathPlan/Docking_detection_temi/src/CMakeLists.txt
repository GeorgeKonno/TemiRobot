cmake_minimum_required(VERSION 2.8.3)

project(docking_debug)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS

cv_bridge

image_transport

roscpp

sensor_msgs

std_msgs

tf

)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

set(SRC   ${PROJECT_SOURCE_DIR}/PointToLineType.cpp
)

add_executable(${PROJECT_NAME} detection.cpp)

target_link_libraries(docking_debug ${catkin_LIBRARIES})
