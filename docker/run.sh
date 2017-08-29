#!/bin/bash
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

if [[ ${USE_NVIDIA} == 1 ]] ; then
  DOCKER_CMD="nvidia-docker"
  DOCKER_OPT=""
else
  DOCKER_CMD="docker"
  DOCKER_OPT="--volume=/dev/dri:/dev/dri:rw \
              --volume=$XAUTH:$XAUTH:rw \
              --env=XAUTHORITY=${XAUTH}"
fi;

"$DOCKER_CMD" run -it ${DOCKER_OPT}\
        --volume=$XSOCK:$XSOCK:rw \
        --env="QT_X11_NO_MITSHM=1" \
        --env="DISPLAY" \
        -v "host_home:$HOME:rw"\
    robotslam
    
