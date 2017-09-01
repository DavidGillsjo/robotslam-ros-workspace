#!/bin/bash
#Usage: attach <container_name>
if [[ "$1" == *"nvidia"* ]]; then
  DOCKER_CMD="nvidia-docker"
else
  DOCKER_CMD="docker"
fi
${DOCKER_CMD} exec -i -t "$1" /bin/bash
