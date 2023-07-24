#!/bin/bash
xhost +
# docker run -it --rm --net=host -v /home/dji/workspace/ffc_4p_ws:/ffc_4p_ws  -v /dev/:/dev/  --privileged -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix  luxonis/depthai-ros:noetic-latest /bin/bash
docker run -it --rm --net=host -v /home/dji/workspace/ffc_4p_ws:/ffc_4p_ws -v /dev/:/dev/  --privileged -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix  oak_ffc:v1.0 /bin/bash