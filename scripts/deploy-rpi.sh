#!/bin/bash

set -eu

# Default values
DATE=`date +%Y%m%d`
USER=`whoami`
VERSION="${USER}_${DATE}"
REMOTE_USER="ubuntu"
REMOTE_HOSTNAME="stardust"
ROBOT_NAME="r1"
DO_ROS_RESTART="NO"
DO_INSTALL="NO"
DO_BUILD="NO"
ADDRESS="$REMOTE_USER@$REMOTE_HOSTNAME"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
BUILD_PATH=$( readlink -f ${SCRIPT_DIR}/../rpi-build )

# Parse values
for i in "$@"
do
case $i in
    -u=*|--remote-user=*)
    REMOTE_USER="${i#*=}"
    ADDRESS="$REMOTE_USER@$REMOTE_HOSTNAME"
    shift # past argument=value
    ;;
    -h=*|--remote-hostname=*)
    REMOTE_HOSTNAME="${i#*=}"
    ADDRESS="$REMOTE_USER@$REMOTE_HOSTNAME"
    shift # past argument=value
    ;;
    -v=*|--version=*)
    VERSION="${i#*=}"
    shift # past argument=value
    ;;
    --robot-name=*)
    ROBOT_NAME="${i#*=}"
    shift # past argument=value
    ;;
    -r|--ros-restart)
    DO_ROS_RESTART="YES"
    shift # past argument=value
    ;;
    -i|--install-on-startup)
    DO_INSTALL="YES"
    shift # past argument=value
    ;;
    -b|--build)
    DO_BUILD="YES"
    shift # past argument=value
    ;;
    *)
        echo "Usage deploy-rpi.sh "
        echo "  --remote-user=user         (default: ubuntu)"
        echo "  --remote-hostname=hostname (default: stardust)"
        echo "  --version=stardustX        (default: username_current_date)"
        echo "  --robot-name=rX            (default:r1)"
        echo "  --ros-restart              (default : no)"
        echo "  --install-on-startup       (default: no)"
        echo "  --build                    (default:no)" 

        exit 1 
    ;;
esac
done

# Show parameters
echo "REMOTE USER                = ${REMOTE_USER}"
echo "REMOTE HOSTNAME            = ${REMOTE_HOSTNAME}"
echo "VERSION                    = ${VERSION}"
echo "ROBOT NAME                 = ${ROBOT_NAME}"
echo "BUILD BEFORE UPLOAD        = ${DO_BUILD}"
echo "INSTALL AS STARTUP SERVICE = ${DO_INSTALL}"
echo "ROS RESTART AFTER UPLOAD   = ${DO_ROS_RESTART}"

# Deploy script
echo "=== Deploying scripts"
rsync -e ssh -avzr ${SCRIPT_DIR} ${ADDRESS}:/home/${REMOTE_USER}/${VERSION}
echo "=== Scripts deployed"

# Stop current ros process if started
if [[ "$DO_ROS_RESTART" == "YES" ]]; then
    echo "=== Stopping any ROS process on robot"
    ssh ${ADDRESS} "/home/${REMOTE_USER}/${VERSION}/scripts/stop.sh"
    echo "=== ROS processes on robot have been stopped"
fi

# Build
if [[ "$DO_BUILD" == "YES" ]]; then
    echo "=== Starting build process"
    ${SCRIPT_DIR}/build-rpi.sh
    echo "=== Build process done"
fi

# Deploy code
echo "=== Deploying a new version..."
rsync -e ssh -avzr ${BUILD_PATH}/install ${ADDRESS}:/home/${REMOTE_USER}/${VERSION}
echo "=== New version deployed"

# Install this version as a startup service
if [[ "$DO_INSTALL" == "YES" ]]; then
    echo "=== Creating startup service (you'll be prompted to give sudo password)"
    cat > /tmp/ros_${VERSION}.service << EOL
[Unit]
Description=Stardust ROS
After=network-online.target

[Service]
Type=forking
 
User=${REMOTE_USER}
Group=${REMOTE_USER}
UMask=007
 
ExecStart=/home/${REMOTE_USER}/stardust/${VERSION}/script/start.sh ${ROBOT_NAME}
ExecStop=/home/${REMOTE_USER}/stardust/${VERSION}/script/stop.sh

[Install]
WantedBy=default.target
EOL
    rsync -e ssh -avz /tmp/ros_${VERSION}.service ${ADDRESS}:/home/${REMOTE_USER}/${VERSION}

    # Disable any other version
    ssh ${ADDRESS} "/home/${REMOTE_USER}/${VERSION}/scripts/stop.sh remove_services"

    # Install new version
    ssh ${ADDRESS} "sudo systemctl enable /home/${REMOTE_USER}/${VERSION}/ros_${VERSION}.service"
    echo "=== Startup service ready"
fi

# Start program
if [[ "$DO_ROS_RESTART" == "YES" ]]; then
    echo "=== Starting ros..."
    ssh ${ADDRESS} "/home/${REMOTE_USER}/${VERSION}/scripts/start.sh ${ROBOT_NAME}"
fi