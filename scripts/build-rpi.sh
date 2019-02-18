#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
PROJECT_DIR=${SCRIPT_DIR}/..
SYSROOT_DIR=$( readlink -f "${PROJECT_DIR}/rpi-sysroot")

source ${SYSROOT_DIR}/opt/ros/melodic/setup.bash

cd ${PROJECT_DIR}/ros

#-DCMAKE_VERBOSE_MAKEFILE=on 
${SYSROOT_DIR}/opt/ros/melodic/bin/catkin_make -DCMAKE_TOOLCHAIN_FILE=${SCRIPT_DIR}/rpi3b+.toolchain -DSYSROOT_DIR=${SYSROOT_DIR}