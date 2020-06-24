cmake_minimum_required(VERSION 3.4)

project(typing-8 CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

## Declare a catkin package
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(examples_common STATIC
  src/examples_common.cpp
)
target_include_directories(examples_common PRIVATE /usr/include/eigen3)
target_link_libraries(examples_common PUBLIC Franka::Franka)

add_executable(controller src/main.cpp src/lift_controller.cpp)
target_link_libraries(controller Franka::Franka examples_common ${catkin_LIBRARIES})
target_include_directories(controller PRIVATE /usr/include/eigen3)

add_executable(dummy_controller src/dummy_main.cpp src/lift_controller.cpp)
target_link_libraries(dummy_controller ${catkin_LIBRARIES})
target_include_directories(dummy_controller PRIVATE /usr/include/eigen3)

add_executable(measuring_forces src/measuring_forces.cpp)
target_link_libraries(measuring_forces Franka::Franka examples_common ${catkin_LIBRARIES})
target_include_directories(measuring_forces PRIVATE /usr/include/eigen3)

add_executable(playground src/playground.cpp)
target_link_libraries(playground examples_common ${catkin_LIBRARIES})
target_include_directories(playground PRIVATE /usr/include/eigen3)
