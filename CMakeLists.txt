cmake_minimum_required(VERSION 3.5)
project(serp_teleop)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake_python REQUIRED)
find_package(flatland_msgs REQUIRED)

## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/world"
DESTINATION "share/${PROJECT_NAME}")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/launch"
DESTINATION "share/${PROJECT_NAME}")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/rviz"
DESTINATION "share/${PROJECT_NAME}")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/serp_teleop"
DESTINATION "share/${PROJECT_NAME}")

ament_package(

)