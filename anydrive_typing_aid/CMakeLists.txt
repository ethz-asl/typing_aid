cmake_minimum_required(VERSION 2.8.3)
project(anydrive_typing_aid)

add_definitions(-std=c++11 -Wall -Werror)

find_package(catkin REQUIRED
  COMPONENTS
    anydrive_ethercat_ros
)

catkin_python_setup()

catkin_package(
  # INCLUDE_DIRS
  #   include
  # LIBRARIES
  #   ${PROJECT_NAME}
  CATKIN_DEPENDS
    anydrive_ethercat_ros
)

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
# )


