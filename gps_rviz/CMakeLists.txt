cmake_minimum_required(VERSION 2.8.3)
project(gps_rviz)

find_package(catkin REQUIRED COMPONENTS roscpp rospy)
find_package(Boost REQUIRED COMPONENTS thread)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gps_rviz
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)





add_executable(gps_rviz src/gps_rviz.cpp)
target_link_libraries(gps_rviz ${catkin_LIBRARIES})
