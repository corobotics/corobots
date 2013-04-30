#!/bin/bash

# Run this as root, from the directory containing it!
#
# USAGE: sudo ./install.bash [network_interface] [ros_release]
#
# Where network_interface defaults to whatever active wireless interface
# you have, and ros_release defaults to the latest installed.

interface=$(iwconfig 2>/dev/null | awk '{print $1}' | head -n1)

if [ $# -gt 0 -a "$1" != "" ]; then
    interface=$1
fi

if [ "$interface" = "" ]; then
    echo "Couldn't detect interface."
    exit 1
else
    echo "Using network interface $interface."
fi

release=$(ls /opt/ros/ | tail -n1)

if [ $# -gt 1 -a "$2" != "" ]; then
    release=$2
fi

if [ "$release" = "" ]; then
    echo "Couldn't find ROS release installed."
    exit 1
else
    echo "Using ROS release $release."
fi

if ! cp ./52-corobot.rules /etc/udev/rules.d/; then
    echo "Error copying udev rules; are you root?"
    exit 1
fi

# checks if turtlebot user+group exists, if it doesn't, then it creates a turtlebot daemon.

if ! grep "^corobot:" /etc/group >/dev/null 2>&1; then
    echo "Group corobot does not exist, creating."
    groupadd corobot
fi

if ! id -u corobot >/dev/null 2>&1; then
    echo "User corobot does not exist, creating and adding it to groups corobot and sudo."
    useradd -g corobot corobot
    usermod corobot -G sudo
    if [ ! -e /home/corobot ]; then
        echo "Corobot home directory was not created, creating."
        mkdir /home/corobot
        chown corobot:corobot /home/corobot
    fi
fi

cat ./corobot-start | sed "s/wlan0/$interface/g" | sed "s/fuerte/$release/"g > /usr/sbin/corobot-start
cat ./corobot-stop | sed "s/wlan0/$interface/g" | sed "s/fuerte/$release/"g > /usr/sbin/corobot-stop
cat ./corobot.conf | sed "s/wlan0/$interface/g" > /etc/init/corobot.conf
chmod +x /usr/sbin/corobot-start
chmod +x /usr/sbin/corobot-stop

mkdir -p /etc/ros
mkdir -p /etc/ros/$release

echo ". /opt/ros/$release/setup.bash;" > /etc/ros/setup.bash
