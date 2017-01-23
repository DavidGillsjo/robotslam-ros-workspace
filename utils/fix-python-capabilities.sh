setcap cap_net_raw+ep /usr/bin/python2.7

#Adding capabilities like this disables LD_... env. variables. So we have to workaround this:
echo "/opt/ros/kinetic/lib" > /etc/ld.so.conf.d/ros.conf
ldconfig
