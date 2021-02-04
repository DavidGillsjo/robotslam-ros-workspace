#!/bin/bash
#Usage: [ENV_OPTS] ./run [CMD] [ARGS]

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -


if [ "${USE_NVIDIA}" == 1 ] ; then
  NVIDIA_ARGS="--gpus all"
  DOCKER_NAME="robotslam_nvidia"
else
  NVIDIA_ARGS=""
  DOCKER_NAME="robotslam_intel"
  MY_DOCKER_OPT="--volume=/dev/dri:/dev/dri:rw \
              --volume=$XAUTH:$XAUTH:rw \
              --env=XAUTHORITY=${XAUTH}"
fi


docker run --rm -it ${MY_DOCKER_OPT} ${DOCKER_OPT}\
        --name=${DOCKER_NAME}\
        --volume=$XSOCK:$XSOCK:rw \
        --network=${NETWORK-bridge}\
        --env="QT_X11_NO_MITSHM=1" \
        --env="DISPLAY" \
        --env="TZ=${TZ-Europe/Stockholm}" \
        -v "${HOME}:/host_home:rw"\
        -v "${ROSBAG-/tmp/rosbag}:/rosbag:rw"\
        robotslam "$@"
