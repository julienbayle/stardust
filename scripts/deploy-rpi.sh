#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters, use syntax : $0 <user>@<address>"
    exit 1
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

BUILD_PATH=$( readlink -f ${SCRIPT_DIR}/../rpi-build )
ADDRESS=$1

# Build
${SCRIPT_DIR}/build-rpi.sh

# Deploy
rsync -e ssh -avzr ${BUILD_PATH}/install ${ADDRESS}:/home/ubuntu/stardust