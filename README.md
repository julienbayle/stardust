StarBaby
================================

## Introduction

This is the main program of the StarBaby robot for Eurobot 2018.
It is based on ROS kinetic and a Raspberry Pi 3

## Installation on Raspberry Pi 3

Ubuntu 16.04 has been selected because this Linux distribution is compatible
With raspberry Pi 3 and ROS ARM packages are availables.

On raspbian (previously used), rosdep does not find all packages. 
In consequence, a lot of packages must be installed and compiled by hand. It was painful.

Two distributions of Ubuntu for Raspberry 3 were tested.
The first one was from https://wiki.ubuntu.com/ARM/RaspberryPi
It was a failure (apt-upgrade bugs, failed boots avec apt-get install, wifi unstable, slow ping, ...)

The second one was the good one : https://ubuntu-mate.org/download/
Everything went fine.

Note : This installation is quite a long process. About two to three hours. 

### Base image : Ubuntu 16.04

Download ubuntu 16.04 image from https://ubuntu-mate.org/download/
Then, write the image on a SD CARD (Mac OS version) :

```bash
brew install xz
diskutil umountDisk /dev/disk2
xzcat ubuntu-mate-16.04.2-desktop-armhf-raspberry-pi.img.xz | sudo dd bs=4m of=/dev/rdisk2
```

Then put the SD card into the Raspberry Pi and turn it on.
Connect an ethernet cable to the Raspberry, an HDMI cable and a mouse (keyboard is optionnal)

On the screen, complete the startup wizard. Fine.

Ubuntu MATE doesn't provide SSH server by default.
Let's add it.

```bash
sudo apt-get install openssh-server
sudo systemctl enable ssh.service
sudo ufw allow 22
sudo systemctl restart ssh.service
```

### System update

By default, system partitions are not well sized. Sytem update is impossible.
Use gparted to change /boot size (128mo for example)

```bash
sudo apt-get install gparted
sudo gparted
```

Then, update the system and reboot to check that everything goes fine
(with some ubuntu images, raspberry does not restart after upgrade...)

```bash
sudo apt-get update
sudo apt-get upgrade
sudo reboot
```

### Network and WIFI connexion

Using the desktop interface, connect your raspberry to the wifi network
You can also do it with ssh, using *nmtui* tool

Note your IP and reboot to check that WIFI goes up at startup.
Unplug the raspberry pi from ethernet while halting.

```bash
ifconfig
sudo reboot
```

### ROS installation

Source : http://wiki.ros.org/Installation/UbuntuARM

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install -y ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt-get install ros-kinetic-joy ros-kinetic-robot-localization ros-kinetic-ros-controllers ros-kinetic-ros-control
```

### StarBaby ROS packages

```bash
git clone https://github.com/julienbayle/starbaby
cd starbaby/StarBabyROS/src
wstool update
cd ..
rosdep install --from-paths src --ignore-src --rosdistro kinetic -r -y
sudo fallocate -l 512m /file.swap
sudo chmod 600 /file.swap 
sudo mkswap /file.swap 
sudo swapon /file.swap
catkin_make
sudo swapoff /file.swap
source devel/setup.bash
```

### Raspberry configuration for the robot

#### LIDAR (UART) and GPIO

The LIDAR use the raspberry serial port (3,3V / 8N1 / 115220). However, this one is used by bluetooth on raspberry pi 3. The console is also using the serial port when bluetooth is disabled. So we have to disable these functionalities. Moreover, our our must be authorized to use the serial port

```bash
sudo usermod -a -G dialout pi
sudo systemctl disable hciuart
```

Add to /boot/config.txt :

```
dtoverlay=pi3-disable-bt

```
Remove these from /boot/cmdline.txt

```
console=serial0,115200
```

Many ROS nodes use Python GPIO for raspberry. As there is not rosdep for this package, we need to install it manualy :

```bash
sudo apt-get install python-pip python-dev
sudo pip install RPi.GPIO 
sudo reboot
```

Now, check that LIDAR can work :

```bash
cd starbaby/StarBabyROS/
source devel/setup.bash
roslaunch xv_11_laser_motor_control xv_11_laser.launch &
rostopic echo /rpms
```

If the LIDAR works, it starts rotating at a regulated speed and emit on topic *rpms*, speed values 
(value must be arount 300 +/-10) 

#### Activate I2C and SPI using raspi-config.

Robot eyes uses SPI and IMU uses I2C, so we have to activate both protocols.
For this, we use *raspi-config*

```bash
sudo raspi-config 
sudo reboot
```

Check that IMU is found over I2C

```bash
sudo i2cdetect -y 1
```

Should return :

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- 1e -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- 53 -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --
```

#### Add python library for LED MATRIX

*starbaby_led_matrix* package depends on *luma.led_matrix* python package

```bash
pip install luma.led_matrix
```
 
### Run 

To check everything is fine, start the main launch file

```bash
roslaunch starbaby starbaby.launch
```

### Startup script

In order that ROS code starts automaticaly with boot process, just type:

```bash
/home/pi/starbaby/StarBabyService/install.sh
```

## Configure a development computer

### Install ROS

```bash
sudo apt-get update
sudo apt-get install \
	ros-kinetic-desktop-full \ 
	ros-kinetic-gazebo-plugins \ 
	ros-kinetic-hector-gazebo-plugins \ 
	ros-kinetic-gazebo-ros-control
rosdep init
sudo rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Configure ROS workspace for the project

```bash
git clone https://github.com/julienbayle/starbaby
cd starbaby/StarBabyROS/src
wstool update
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

### Connect an XBOX 360 receiver

Connect your XBOX 360 reciever to your computer.
Turn on your XBOX 360 pad, wait it be available on /dev/input/js0

### Run Gazebo robot simulation

```bash
roslaunch starbaby_gazebo starbaby_gazebo.launch
```

By default, use the pad to control the robot.

To activate the automatic mode, push the "start" button on the pad.

Then send a 2D nav goal via RVIZ to the robot and the navigation stack does the magic.

To take back the control with the pad, push the "back" button 


## Debug the robot from a remote computer

### On the Raspberry Pi

```bash
export ROS_MASTER=http://RASPBERRY_IP:11311
export ROS_IP=RASPBERRY_PI
roslaunch starbaby starbaby.launch
```

### On the remote computer

For example, to run RVIZ

```bash
export ROS_MASTER=http://RASPBERRY_IP:11311
export ROS_IP=REMOTE_COMPUTER_PI
rosrun rviz rviz
```

## VIM autocomplete for ROS

Install dependencies (VIM autocomplet and VIM-Plug)

```bash
sudo apt-get install vim-nox-py2
curl -fLo ~/.vim/autoload/plug.vim --create-dirs https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim
```

Edit VIMRC (~/.vimrc) and add this :
 
```
let g:ycm_semantic_triggers = {
			\   'roslaunch' : ['="', '$(', '/'],
			\   'rosmsg,rossrv,rosaction' : ['re!^', '/'],
			\ }

call plug#begin('~/.vim/plugged')

" VIM ROS
Plug 'taketwo/vim-ros'

" You complete me
Plug 'Valloric/YouCompleteMe'

" Initialize plugin system
call plug#end()
```
	
Open VIM and type ":PluginsInstall"

To finish *YouCompleteMe* install, we have to compile some dependencies by hand :

```bash
cd .vim/plugged/YouCompleteMe/
sudo swapon /file.swap
./install.py
sudo swapoff /file.swap 
```







