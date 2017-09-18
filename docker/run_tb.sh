#!/bin/bash
# Script to run on the turtlebot laptop

# TODO: Pull docker image. No need to build it here.

# Find USB device to store data on.
export USBKEYS=($(
    for blk in $(lsblk -ndo name) ; do {
        udevadm info --query=all --name "/dev/$blk" |\
        grep -q ID_BUS=usb &&
            printf 'findmnt %s -no TARGET ;'\
                "/dev/$blk" /dev/"$blk"[0-9]
        } ; done 2>&- |. /dev/stdin
))
echo "$USBKEYS"
export STICK
case ${#USBKEYS[@]} in
    0 ) echo "No USB stick found. Storing rosbags locally" ;;
    1 ) STICK=$USBKEYS; echo "STICK=$USBKEYS" ;;
    * )
    STICK=$(
    bash -c "$(
        echo -n  dialog --menu \
            \"Choose wich USB stick to store rosbags on\" 22 76 17;
        for dev in ${USBKEYS[@]} ;do
            echo -n \ $dev \"$(
                sed -e s/\ *$//g </sys/block/$dev/device/model
                )\" ;
            done
        )" 2>&1 >/dev/tty
    )
    ;;
esac

#Create directory and set env variable
ROSBAG_ROOT=${STICK-${HOME}}
ROSBAG="${ROSBAG_ROOT}/rosbags"
mkdir -p ${ROSBAG}

# Start docker container
./run.sh
