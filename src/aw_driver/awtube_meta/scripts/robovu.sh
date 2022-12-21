#!/bin/bash

echo "* Running RoboVu container *"

docker run -it --rm --privileged --net=host automationware/robovu $1 $2 $3