#!/bin/bash

if [ -f /tmp/ros.pid ]; then
	echo "Merci d'arrêter de ROS avant de lancer une mise à jour"
fi

echo "Updating ROS source code"
cd /home/pi/stardust
git pull

cd /home/pi/stardust/ros
echo "Updating WSTOOL dependencies (.rosinstall)"
wstool update -t src
echo "Updating ROSDEPS (package dependencies)"
rosdep install --from-paths src --ignore-src --rosdistro kinetic -r -y
catkin_make
source devel/setup.bash