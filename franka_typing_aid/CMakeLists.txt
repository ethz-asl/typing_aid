cmake_minimum_required(VERSION 3.4)

project(franka_typing_aid CXX)

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

add_executable(franka_lifting_controller src/main.cpp src/LiftController.cpp
                          src/LiftControllerJointPosition.cpp
                          src/LiftControllerCartesianImpedance.cpp)
target_link_libraries(franka_lifting_controller Franka::Franka examples_common ${catkin_LIBRARIES})
target_include_directories(franka_lifting_controller PRIVATE /usr/include/eigen3 ${catkin_INCLUDE_DIRS})

# add_executable(franka_lifting_dummy_controller src/dummy_main.cpp src/LiftController.cpp)
# target_link_libraries(franka_lifting_dummy_controller ${catkin_LIBRARIES})
# target_include_directories(franka_lifting_dummy_controller PRIVATE /usr/include/eigen3 ${catkin_INCLUDE_DIRS})

add_executable(franka_measuring_forces src/measuring_forces.cpp)
target_link_libraries(franka_measuring_forces Franka::Franka examples_common ${catkin_LIBRARIES})
target_include_directories(franka_measuring_forces PRIVATE /usr/include/eigen3)

add_executable(playground src/playground.cpp)
target_link_libraries(playground examples_common ${catkin_LIBRARIES})
target_include_directories(playground PRIVATE /usr/include/eigen3)
