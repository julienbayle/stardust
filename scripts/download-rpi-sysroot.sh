#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters, use syntax : $0 <address>"
    exit 1
fi

export ADDRESS=$1

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

export SYSROOT_PATH=$( readlink -f ${SCRIPT_DIR}/../rpi-sysroot )

echo "Rsync to path ${SYSROOT_PATH}"

mkdir -p ${SYSROOT_PATH}
rsync -e ssh -avzr --progress ubuntu@${ADDRESS}:/lib ubuntu@${ADDRESS}:/usr ubuntu@${ADDRESS}:/opt ${SYSROOT_PATH}

if [ $? -ne 0 ]; then
    echo "Error while rsyncing sysroot"
    exit 1
fi

rsync -e ssh -avzr --progress ubuntu@${ADDRESS}:/etc/alternatives ${SYSROOT_PATH}/etc

if [ $? -ne 0 ]; then
    echo "Error while rsyncing sysroot"
    exit 1
fi

echo "Patching symbolic links"
 
ROOTFS=$(readlink -f ${SYSROOT_PATH})
cd $ROOTFS/usr/lib
find . -type l | while read i;
do qualifies=$(readlink $i | grep '^\(/usr\)\?/lib')
   if [ -n "$qualifies" ]; then
       newPath=$(readlink $i | grep '^\(/usr\)\?/lib' | xargs echo $ROOTFS | sed -e s/\\s//g)
       rm $i;
       ln -s $newPath $i;
   fi
done
find . -type l | while read i;
do qualifies=$(readlink $i | grep '^\(/etc\)\?/alternatives')
   if [ -n "$qualifies" ]; then
       newPath=$(readlink $i | grep '^\(/etc\)\?/alternatives' | xargs echo $ROOTFS | sed -e s/\\s//g)
       rm $i;
       ln -s $newPath $i;
   fi
done
cd $ROOTFS/etc/alternatives
find . -type l | while read i;
do qualifies=$(readlink $i | grep '^\(/usr\)\?/lib')
   if [ -n "$qualifies" ]; then
       newPath=$(readlink $i | grep '^\(/usr\)\?/lib' | xargs echo $ROOTFS | sed -e s/\\s//g)
       rm $i;
       ln -s $newPath $i;
   fi
done
 
echo "Patching cmake scripts"
 
for i in `find ${SYSROOT_PATH}/opt/ros/melodic/share/ -name *.cmake`; do sed -i "s,/opt/,"${SYSROOT_PATH}"/opt/,g" $i; done
for i in `find ${SYSROOT_PATH}/opt/ros/melodic/share/ -name *.cmake`; do sed -i "s,/usr/,"${SYSROOT_PATH}"/usr/,g" $i; done
for i in `find ${SYSROOT_PATH}/usr -name *.cmake`; do sed -i "s,/usr/,"${SYSROOT_PATH}"/usr/,g" $i; done

sed -i "s,/usr/lib/aarch64-linux-gnu/,,g" ${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libpthread.so
sed -i "s,/lib/aarch64-linux-gnu/,,g" ${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libpthread.so
sed -i "s,/usr/lib/aarch64-linux-gnu/,,g" ${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libc.so
sed -i "s,/lib/aarch64-linux-gnu/,,g" ${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libc.so

echo "Sysroot ok"
