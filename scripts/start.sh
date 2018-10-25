#!/bin/bash

# Get IP for OS
BASE=$(dirname "$0")
IP=`ip addr | grep 'inet .* wlan0' | awk '{print $2}' | cut -f1 -d'/'`

# If network is down then works in isolated mode
if [ -z "$IP" ]; then 
	IP="127.0.0.1"
fi

# Get robot identifier
if [ -f $BASE/../scripts/robot.id ]; then
	ID=`cat $BASE/../scripts/robot.id`
else
	echo "Merci de créer un fichier robot.id"
	exit 1
fi

# Start ROS
source $BASE/../ros/devel/setup.bash
export ROS_IP=$IP
export ROS_MASTER=http://$IP:11311

echo "Starting ROS on $IP for robot $ID"
roslaunch --pid=/tmp/ros.pid sd_main $ID.launch & 