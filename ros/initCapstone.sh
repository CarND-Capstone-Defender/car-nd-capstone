#!/bin/sh
cd src
rm CMakeLists.txt
catkin_init_workspace
logdirectory="/capstone/log"
if [ ! -d "${logdirectory}" ]
then
    mkdir -p "${logdirectory}"
fi
