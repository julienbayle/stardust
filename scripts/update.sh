#!/bin/bash

if [ -f /tmp/ros.pid ]; then
        echo "Merci d'arrêter de ROS avant de lancer une mise à jour"
fi

PROJECTDIR=$(dirname "$0")
echo $PROJECTDIR
PROJECTDIR=`cd $PROJECTDIR/.. && pwd`
echo "Updating ROS source code ($PROJECTDIR)"
cd $PROJECTDIR
git pull

cd $PROJECTDIR/ros/
echo "Updating WSTOOL dependencies (.rosinstall)"
wstool update -t src
echo "Updating ROSDEPS (package dependencies)"
rosdep install --from-paths src --ignore-src --rosdistro melodic -r -y
catkin_make
