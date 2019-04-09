# Installation sur Raspberry Pi 3 modèle B ou B+

Ubuntu 18.04 doit être utilisé car cette distribution et version de linux propose directement les dépôts ARM de ROS Melodic, compatible sur raspberry pi 3 B et B+.

L'utilisation de Raspbian a été testé mais l'outil "rosdep" ne trouve pas dans les dépôts de cette distribution tous les paquets nécessaires. Aussi, il faut les compiler manuellement, ce qui est super pénible et prend beaucoup de temps.

**Compter à peu près 2 heures pour réaliser cette installation**

## Installation de la distribution Ubuntu 18.04 sur clé USB

Récupérer l'image "Ubuntu 18.04" depuis http://cdimage.ubuntu.com/releases/18.04/beta/
Ecrire l'image sur une clé USB

### Ubuntu
```bash
xzcat ubuntu-18.04-beta-preinstalled-server-arm64+raspi3.img.xz | sudo dd bs=4M of=/dev/sdb
```

### Mac OS 
```bash
brew install xz
diskutil umountDisk /dev/disk2
xzcat ubuntu-18.04-beta-preinstalled-server-arm64+raspi3.img.xz | sudo dd bs=4m of=/dev/rdisk2
```

Mettre la carte dans le Raspberry Pi et l'allumer. Connecter une câble ethernet, un écran via le port HDMI et un clavier.

## Boot sur la clé USB

Branchez la clé USB sur un PC, et éditez le fichier cmdline.txt sur la partition "system-boot":
```
console=tty0 console=ttyS1,115200 root=LABEL=writable rw elevator=deadline fsck.repair=yes net.ifnames=0 cma=64M rootwait rootdelay=10
```

Creez un fichier "boot.scr.txt" avec le contenu suivant:
```
setenv fdt_addr_r 0x03000000
fdt addr ${fdt_addr_r}
fdt get value bootargs /chosen bootargs
setenv kernel_addr_r 0x01000000
setenv ramdisk_addr_r 0x03100000
fatload usb 0:1 ${kernel_addr_r} vmlinuz
fatload usb 0:1 ${ramdisk_addr_r} initrd.img
setenv initrdsize $filesize
booti ${kernel_addr_r} ${ramdisk_addr_r}:${initrdsize} ${fdt_addr_r}
```

Remplacez également le contenu du fichier "/etc/flash-kernel/bootscript/bootscr.rpi3" sur la partition "writable" par le contenu ci-dessus.

Entrez les commandes suivantes:
```shell
$ sudo apt-get install u-boot-tools
$ mkimage -A arm -O linux -T script -C none -n boot.scr -d boot.scr.txt boot.scr
```

Remplacez le fichier "boot.scr" sur la partition "system-boot".

Ajouter une ligne au fichier "/etc/default/raspi3-firmware":
```
ROOTPART=LABEL=writable
```

## Désactivation des mises à jour automatiques

Editer le fichier "/etc/apt/apt.conf.d/20auto-upgrades" et remplacez la ligne:
```
APT::Periodic::Update-Package-Lists "1";
APT::Periodic::Unattended-Upgrade "1";
```
par
```
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";
```

## Mise à jour du système

Mise à jour du système et redémarrage :

```bash
sudo apt-get update
sudo apt-get upgrade
sudo reboot
```

## Installation de NetworkManager

```bash
sudo apt install network-manager
sudo systemctl start NetworkManager
sudo nmtui
```

## Installation de ROS

Source : http://wiki.ros.org/Installation/UbuntuARM

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install -y ros-melodic-desktop-full git python-wstool vim
sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Installation et compilation du code source du robot

```shell
$ cd ~
$ git clone https://github.com/julienbayle/stardust.git
```

### Deploiement du projet sans cross compilation :

La première compilation du projet nécessite l'activation de la SWAP car la mémoire du Raspberry est insuffisante. Pour compiler une simple modification locale du code, ceci n'est pas nécessaire ensuite.
	
Création d'un fichier SWAP de 512Mo (Ce fichier sera conservé car il re-servira souvent par la suite) :
	
```bash
sudo fallocate -l 512m /512m.swap
sudo chmod 600 /512m.swap 
sudo mkswap /512m.swap 
```

Compilation du projet ROS :
	
```bash
sudo swapon /512m.swap
~/stardust/scripts/update.sh
sudo swapoff /512m.swap
```

### Deploiement du projet par cross compilation :

Téléchargement des paquets nécessaires:

```shell
$ cd ~/stardust/ros
$ rosdep install --from-paths src --ignore-src --rosdistro melodic -r -y
```

Puis :

[Cross compilation et déploiement sur le raspberry](cross_compilation_raspberry.md).

### Configuration du Raspberry

### Lancer le programme (premier démarrage)

Tester que le projet démarre (remplacer r1 par r2 pour le robot secondaire) :
	
```bash
cd ~/stardust/ros/
source devel/setup.bash
roslaunch sd_main r1.launch
```

### LIDAR (UART)

Le LIDAR est connecté au port série du raspberry (3,3V / 8N1 / 115220). Cependant, celui-ci est utilisé par le module Bluetooth présent sur la carte et par la console du raspberry. Il faut désactiver ces deux options pour libérer le port série puis autoriser l'utilisateur courant à utiliser le port série.

A ajouter dans le fichier /boot/config.txt :

```
dtoverlay=pi3-disable-bt
```

A supprimer du fichier /boot/cmdline.txt

```
console=serial0,115200
```

Enfin, il faut autoriser notre utilisateur à utiliser le port série (remplacer "r1" par le nom de votre ut)

```bash
USER=`whoami`
sudo usermod -a -G dialout $USER
sudo systemctl disable hciuart
sudo reboot
```

[Connecter le LIDAR au robot](https://github.com/julienbayle/xv_11_lidar_raspberry) pour vérifier qu'il fonctionne.

```bash
cd ~/stardust/ros/
source devel/setup.bash
roslaunch xv_11_lidar_raspberry xv_11_lidar_raspberry.launch port:=/dev/serial0
```

Si le LIDAR fonctionne, il doit se mettre à tourner et sa vitesse de rotation doit être régulée. La vitesse de rotation est émise sur un topic incluant le terme *rpms*. La vitesse est régulée autour de 300 rotations par minute soit 5 tours par seconde.

### Activation de l'I2C et du SPI

Activer ces protocoles via l'outil **raspi-config** :

```bash
sudo raspi-config 
sudo reboot
```

Si une IMU est connectée au robot via I2C, il est possible de vérifier qu'elle est bien disponible sur le port I2C et que celui-ci marche bien :

```bash
i2cdetect -y 1
```

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

On voit ici les 3 puces de l'IMU (gyromètre, acceleromètre et magnétomètre)

### Support des pavés LED

Si le robot est connecté via SPI à un ou plusieurs pavés LED, il faut ajouter la librairie nécessaire (pas de paquet rosdep de disponible)

```bash
sudo pip install luma.led_matrix
```

## Ajout d'une manette

Les manettes XBOX360 et PS4 sont nativement prises en charge par le noyau linux. Il suffit donc de les lier au raspberry.

Connecter le recepteur au raspberry (clé USB bluetooth pour manette PS4 et récepteur radio pour manette XBOX).

Connecter la manette et effectuer l'appairage.

Vérifier que la manette fonctionne :

```bash
sudo apt install joystick 
jstest /dev/input/js0
```

## Scripts de démarrage et de mise à jour

Le répertoire *scripts* contient des scripts pour lancer, mettre à jour et arrêter ROS sur le robot.

Pour identifier le robot, il faut tout d'abord créer un fichier *robot.id* dans le dossier script et mettre dedant **r1** ou **r2** selon qu'il s'agit d'une installation pour le robot principal ou secondaire.

```bash
echo "r1" > ~/stardust/scripts/robot.id
```

Activation du démarrage automatique de ROS au démarrage du Raspberry PI :

```bash
~/stardust/scripts/install.sh
```

Pour démarrer ROS manuellement (En tâche de fond) :

```bash
~/stardust/scripts/start.sh
```

Pour arrêter ROS manuellement :

```bash
~/stardust/scripts/stop.sh
```

Pour mettre à jour ROS :

```bash
~/stardust/scripts/update.sh
```

## Accès au robot depuis un PC distant

Exemple pour lancer rviz depuis un poste distant (ROS doit être lancé sur le robot via le script ci-dessus et le robot et l'ordinateur doivent être sur le même réseau) :

```bash
export ROS_MASTER_URI=http://RASPBERRY_IP:11311
export ROS_IP=COMPUTER_IP
rosrun rviz rviz
```
