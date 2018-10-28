#!/bin/bash

if [ -f /tmp/ros.pid ]; then
	echo "Stopping ROS..."
	kill -n SIGINT `cat /tmp/ros.pid`
	rm /tmp/ros.pid
	echo "ROS stopped"
else
	echo "ROS is stopped or was not started by this script"
fi
