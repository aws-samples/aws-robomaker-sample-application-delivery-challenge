#!/bin/bash

#download from ROBITIS-GIT
if [ -d ./robot_ws/src ]; then
    git clone -b add-table-and-camera https://github.com/ROBOTIS-JAPAN-GIT/turtlebot3.git robot_ws/src/turtlebot3

    #remove unnesesary files
    rm -rf robot_ws/src/turtlebot3/turtlebot3_bringup
    rm -rf robot_ws/src/turtlebot3/turtlebot3_teleop
    rm -rf robot_ws/src/turtlebot3/turtlebot3
fi 

if [ -d ./simulation_ws/src ]; then

    git clone -b add-table-and-camera https://github.com/ROBOTIS-JAPAN-GIT/turtlebot3.git simulation_ws/src/turtlebot3

    #remove unnesesary files
    rm -rf simulation_ws/src/turtlebot3/turtlebot3_bringup
    rm -rf simulation_ws/src/turtlebot3/turtlebot3_teleop
    rm -rf simulation_ws/src/turtlebot3/turtlebot3
fi
