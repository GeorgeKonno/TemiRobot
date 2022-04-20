cmake_minimum_required(VERSION 2.8.3)

project(best_fit)
find_package(Boost COMPONENTS program_options REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

set(CMAKE_CXX_STANDARD 11)

set(SRC   ${PROJECT_SOURCE_DIR}/Main.cpp
)

add_executable(${PROJECT_NAME} Main.cpp)
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
)

