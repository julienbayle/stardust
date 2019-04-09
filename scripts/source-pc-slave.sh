#!/bin/bash

export ROS_MASTER_NAME=stardust
export ROS_MASTER_IP=`getent hosts ${ROS_MASTER_NAME} | awk '{ print $1; }'`
export ROS_IP=`ip route get ${ROS_MASTER_IP} | awk '{print $5; exit}'`
export ROS_MASTER_URI=http://${ROS_MASTER_NAME}:11311