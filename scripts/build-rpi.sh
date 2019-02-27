#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
PROJECT_DIR=${SCRIPT_DIR}/..
SYSROOT_DIR=$( readlink -f "${PROJECT_DIR}/rpi-sysroot")
BUILD_DIR=$( readlink -f "${PROJECT_DIR}/rpi-build")

source ${SYSROOT_DIR}/opt/ros/melodic/setup.bash

mkdir -p ${BUILD_DIR}

#-DCMAKE_VERBOSE_MAKEFILE=on 
${SYSROOT_DIR}/opt/ros/melodic/bin/catkin_make -C ${PROJECT_DIR}/ros -DCMAKE_TOOLCHAIN_FILE=${SCRIPT_DIR}/rpi3b+.toolchain -DSYSROOT_DIR=${SYSROOT_DIR} --build ${BUILD_DIR}/build -DCATKIN_DEVEL_PREFIX=${BUILD_DIR}/devel -DCMAKE_INSTALL_PREFIX=${BUILD_DIR}/install install