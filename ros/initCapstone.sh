#!/bin/sh
cd src
rm CMakeLists.txt
catkin_init_workspace
if [ ! -d "${logdirectory}" ]
then
    mkdir -p "${logdirectory}"
fi
