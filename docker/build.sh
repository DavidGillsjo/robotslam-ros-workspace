#!/bin/bash
docker build\
  -t robotslam\
  --build-arg gid=$(id -g)\
  --build-arg uid=$(id -u)\
  --build-arg home="$HOME"\
  ../.
