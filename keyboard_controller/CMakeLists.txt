cmake_minimum_required(VERSION 2.8.3)
project(keyboard_controller)
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  std_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(keyboard_controller src/keyboard_controller.cpp)

target_link_libraries(keyboard_controller ${catkin_LIBRARIES})
