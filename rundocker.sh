#!/bin/sh
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it --name=defender-container capstone
