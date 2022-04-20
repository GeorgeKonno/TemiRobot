cmake_minimum_required(VERSION 2.8.3)

project(pp_debug)

find_package(OpenCV 3.3 REQUIRED)
#message(${OpenCV_FOUND})
#message(${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME} A_star.cpp)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)
