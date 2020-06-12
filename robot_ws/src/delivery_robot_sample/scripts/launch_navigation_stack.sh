#!/bin/bash

# This script download map from Amazon S3 and launch navigation stack

#Download map from Amazon S3
rosservice call map_download --wait

#Launch navigation stack
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/tmp/map.yaml open_rviz:=false
