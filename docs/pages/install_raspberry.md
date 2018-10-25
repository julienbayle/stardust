# Installation sur Raspberry Pi 3

Ubuntu 16.04 doit être utilisé car cette distribution et version de linux propose directement les dépôts ARM de ROS Kinetic, compatible sur raspberry pi 3.

L'utilisation de Raspbian a été testé mais l'outil "rosdep" ne trouve pas dans les dépôts de cette distribution tous les paquets nécessaires. Aussi, il faut les compiler manuellement, ce qui est super pénible et prend beaucoup de temps.

Deux distributions d'Ubuntu pour Raspberry 3 ont été testées.

La première est issue de la page https://wiki.ubuntu.com/ARM/RaspberryPi. Ce fût une total castastrophe (apt-upgrade bugue, démarrage en échec après un simple apt-get install, wifi non stable, ping lents, ...)

La seconde était la bonne : https://ubuntu-mate.org/download/

**Compter à peu près 2 heures pour réaliser cette installation**

## Installation de la distribution Ubuntu 16.04

Récupérer l'image "Ubuntu 16.04" depuis https://ubuntu-mate.org/download/
Ecrire l'image sur une SD CARD (Mac OS version) :

```bash
brew install xz
diskutil umountDisk /dev/disk2
xzcat ubuntu-mate-16.04.2-desktop-armhf-raspberry-pi.img.xz | sudo dd bs=4m of=/dev/rdisk2
```

Mettre la carte dans le Raspberry Pi et l'allumer. Connecter une câble ethernet, un écran via le port HDMI et une souris. Compléter alors la procédure d'installation qui s'affiche à l'écran.

## Activation du SSH

```bash
sudo apt-get install openssh-server
sudo systemctl enable ssh.service
sudo ufw allow 22
sudo systemctl restart ssh.service
```

## Mise à jour du système

Après l'installation, le partitionnement proposé de la carte SD ne permet pas la mise à jour du système. gparted va être intallé pour remedier à cela.

```bash
sudo apt-get install gparted
sudo gparted
```

Utiliser gparted pour changer la taille de la partition /boot(128mo for example)

Puis, mise à jour du système et redémarrage :

```bash
sudo apt-get update
sudo apt-get upgrade
sudo reboot
```

## Support réseau

Depuis le bureau Ubuntu, connecter votre raspberry au réseau wifi.
(Il est également possible de le faire en ligne de commande ou via SSH en utilisant *nmtui*)

Prendre note de votre IP, débrancher le câble réseau et redémarrer. Vérifier que le raspberry se connecte bien au WIFI au redémarrage.

```bash
ifconfig
sudo reboot
```

## Installation de ROS

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
```

## Installation et compilation du code source du robot

La première compilation nécessite l'activation de la SWAP car la mémoire du Raspberry est insuffisante. Pour les simples mises à jour, ceci n'est pas nécessaire.

```bash
git clone https://github.com/julienbayle/stardust
cd stardust/ros/src
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
## Configuration du Raspberry

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


En ligne de commande :

```bash
sudo usermod -a -G dialout pi
sudo systemctl disable hciuart
sudo reboot
```

### GPIO

Pour l'instant, il n'y a pas de possibilité d'activer cette fonctionnilité via rosdep (pas de paquet permettant d'accéder aux GPIO du raspberry pi). Aussi, il faut ajouter ce support manuellement :

```bash
sudo apt-get install python-pip python-dev
sudo pip install RPi.GPIO 
sudo reboot
```

### Connexion et test du LIDAD

Connecter le LIDAR au robot pour vérifier qu'il fonctionne :

```bash
cd stardust/ros/
source devel/setup.bash
roslaunch xv_11_laser_motor_control xv_11_laser.launch &
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
sudo i2cdetect -y 1
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
pip install luma.led_matrix
```
 

## Ajout d'une manette

Ajouter quelques outils :

```bash
sudo apt install joystick 
```

Connecter le recepteur au raspberry (clé USB bluetooth pour manette PS4 et récepteur radio pour manette XBOX).

Connecter la manette.

Vérifier que la manette fonctionne :

```bash
jstest /dev/input/js0
```

## Scripts de démarrage et de mise à jour

Le répertoire *scripts* contient des scripts pour lancer, mettre à jour et arrêter ROS sur le robot.

Pour identifier le robot, il faut tout d'abord créer un fichier *robot.id* dans le dossier script et mettre dedant **r1** ou **r2** selon qu'il s'agit d'une installation pour le robot principal ou secondaire.

```bash
echo "r1" > /home/pi/stardust/scripts/robot.id
```

Ajout d'un script pour démarrage automatique de ROS :

```bash
/home/pi/stardust/scripts/install.sh
```

Pour démarrer ROS (lancé en tâche de fond) :

```bash
/home/pi/stardust/scripts/start.sh
```

Pour arrêter ROS :

```bash
/home/pi/stardust/scripts/stop.sh
```

Pour mettre à jour ROS :

```bash
/home/pi/stardust/scripts/update.sh
```

## Accès au robot depuis un PC distant

Exemple pour lancer rviz depuis un poste distant
(ROS doit être lancé sur le robot via le script ci-dessus et le robot et l'ordinateur doivent être sur le même réseau)

```bash
export ROS_MASTER=http://RASPBERRY_IP:11311
export ROS_IP=COMPUTER_IP
rosrun rviz rviz
```