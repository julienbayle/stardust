#!/bin/bash

if [ "$#" -ne 2 ]; then
    echo "Illegal number of parameters, use syntax : $0 <user> <address>"
    exit 1
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

BUILD_PATH=$( readlink -f ${SCRIPT_DIR}/../rpi-build )
REMOTE_USER=$1
ADDRESS="$1@$2"

# Build
${SCRIPT_DIR}/build-rpi.sh

# Deploy
rsync -e ssh -avzr ${BUILD_PATH}/install ${ADDRESS}:/home/${REMOTE_USER}/stardust_${USER}
rsync -e ssh -avzr ${SCRIPT_DIR} ${ADDRESS}:/home/${REMOTE_USER}/stardust_${USER}