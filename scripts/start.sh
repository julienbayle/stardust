#!/bin/bash

# Get IP for OS
BASE=$(dirname "$0")
IP=`ip addr | grep 'inet .* wlan0' | awk '{print $2}' | cut -f1 -d'/'`

# If network is board (cable)
if [ -z "$IP" ]; then
        IP=`ip addr | grep 'inet .* brd' | awk '{print $2}' | cut -f1 -d'/'`
fi

# If network is down then works in isolated mode
if [ -z "$IP" ]; then 
	IP="127.0.0.1"
fi

# Get robot identifier
if [ -f $BASE/../scripts/robot.id ]; then
	ID=`cat $BASE/../scripts/robot.id`
elif [ "$#" -eq 1 ]; then
    ID="$1"
else
	echo "Please create a robot.id file or add the robot name as the first parameter of this script"
	exit 1
fi

export ROS_IP=$IP
export ROS_MASTER=http://$IP:11311

# Start ROS
if [ -d $BASE/../install ]; then
    # Start a "cross compiled version"
    source $BASE/../install/setup.bash
elif [ -d $BASE/../ros/devel ]; then
    # Start a "local version"
    source $BASE/../ros/devel/setup.bash
else
    echo "Unable to find ROS project"
    exit 1
fi

echo "Starting ROS on $IP for robot $ID"
roslaunch --pid=/tmp/ros.pid sd_main $ID.launch 2>&1 & 