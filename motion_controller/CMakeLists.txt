cmake_minimum_required(VERSION 2.8.3)
project(motion_controller)
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
    rosconsole
    roscpp
)

set(BOOST_ROOT ~/boost_1_67_0)
set(BOOST_INCLUDEDIR ${BOOST_ROOT}/boost)
set(BOOST_NO_SYSTEM_PATHS ON)
find_package(Boost REQUIRED)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_executable(motion_controller src/motion_controller.cpp src/UDPClient.cpp)

target_link_libraries(motion_controller
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)
