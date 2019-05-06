#!/bin/bash

if [ -f /tmp/ros.pid ]; then
        echo "Merci d'arrêter de ROS avant de lancer une mise à jour"
fi

DO_BUILD="YES"

# Parse values
for i in "$@"
do
case $i in
    --no-build)
    DO_BUILD="NO"
    shift # past argument=value
    ;;
    *)
        echo "Usage update.sh "
        echo "  --no-build              (do not execute catkin-make)"
        exit 1 
    ;;
esac
done

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

if [[ "$DO_BUILD" == "YES" ]]; then
        catkin_make
fi
