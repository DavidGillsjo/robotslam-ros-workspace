#!/bin/bash
#Usage: [ENV_OPTS] ./run [CMD] [ARGS]

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

if [[ ${USE_NVIDIA} == 1 ]] ; then
  DOCKER_CMD="nvidia-docker"
  MY_DOCKER_OPT=""
  DOCKER_NAME="robotslam_nvidia"
else
  DOCKER_CMD="docker"
  MY_DOCKER_OPT="--volume=/dev/dri:/dev/dri:rw \
              --volume=$XAUTH:$XAUTH:rw \
              --env=XAUTHORITY=${XAUTH}"
  DOCKER_NAME="robotslam_intel"
fi;

"$DOCKER_CMD" run --rm -it ${MY_DOCKER_OPT} ${DOCKER_OPT}\
        --name=${DOCKER_NAME}\
        --volume=$XSOCK:$XSOCK:rw \
        --network=${NETWORK-bridge}\
        --env="QT_X11_NO_MITSHM=1" \
        --env="DISPLAY" \
        --env="TZ=Europe/Stockholm" \
        -v "${HOME}:/host_home:rw"\
        -v "${ROSBAG-/tmp/rosbag}:/rosbag:rw"\
        robotslam "$@"
