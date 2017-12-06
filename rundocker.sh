#!/bin/sh
docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it --name=defender-container capstone
