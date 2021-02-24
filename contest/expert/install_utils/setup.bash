#!/bin/bash
# First we want to setup the dependency file so that we can install the pip and apt package
CUSTOM_DEP_SOURCE_LIST_LOCATION=/etc/ros/rosdep/sources.list.d/21-customdependencies.list

sudo touch $CUSTOM_DEP_SOURCE_LIST_LOCATION

if grep -Fxq "yaml file:///${HOME}/custom_dependencies.yaml" $CUSTOM_DEP_SOURCE_LIST_LOCATION
then
    echo "dependency file already setup"
else
    echo "source list not setup"
    echo "yaml file:///${HOME}/custom_dependencies.yaml" | sudo tee -a $CUSTOM_DEP_SOURCE_LIST_LOCATION
fi

DIR_OF_SCRIPT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd $DIR_OF_SCRIPT

if test -f "${HOME}/custom_dependencies.yaml"; then
    echo "template already in place"
else
    echo  "teamplate not found in location"
    cp custom_dependencies_template ${HOME}/custom_dependencies.yaml
fi

echo "DONT FORGET TO RUN FROM WS ROS ROOT WS:  rosdep install -y --from-paths src --ignore-src"
echo "GOING TO RUN ROSDEP UPDATE AND APT UPDATE in 5 SECONDS"
sleep 5

rosdep update

sudo apt-get update
