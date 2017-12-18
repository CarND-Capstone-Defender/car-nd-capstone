#!/bin/bash

# exit on first error
set -e
logdirectory="/capstone/log"
export ROS_LOG_DIR=${logdirectory}
catkin_make
source devel/setup.bash
#roslaunch launch/site.launch
roslaunch launch/styx.launch