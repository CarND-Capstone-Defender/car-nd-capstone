#!/bin/bash

xhost +SI:localuser:root
docker exec -it defender-container ./show.sh

