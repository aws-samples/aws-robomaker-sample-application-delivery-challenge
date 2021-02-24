cmake_minimum_required(VERSION 2.8.3)
project(pose_animation_plugin)

add_definitions(-std=c++11)

find_package(catkin REQUIRED)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(${GAZEBO_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

catkin_package()

add_library(pose_animation_plugin src/pose_animation.cc)
target_link_libraries(pose_animation_plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS pose_animation_plugin LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

