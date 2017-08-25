#!/bin/bash
docker run -it --rm \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  -e DISPLAY=$DISPLAY \
  --name robotslam_c \
  --hostname robotslam_c \
  robotslam
