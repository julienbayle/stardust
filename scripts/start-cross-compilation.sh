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
if [ -f $BASE/robot.id ]; then
	ID=`cat $BASE/robot.id`
else
	echo "Merci de créer un fichier robot.id"
	exit 1
fi

# Start ROS
source $BASE/../install/setup.bash
export ROS_IP=$IP
export ROS_MASTER=http://$IP:11311

echo "Starting ROS on $IP for robot $ID"
roslaunch --pid=/tmp/ros.pid sd_main $ID.launch & 
