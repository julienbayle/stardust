# Installation sur Raspberry Pi 3 modèle B ou B+

La distribution Ubuntu 18.04 doit être utilisée car elle permet d'accéder directement aux dépôts ARM de ROS Melodic, compatible sur raspberry pi 3 (Modèles B et B+). Une clé USB ou une sd card de 8 ou 16 Go est suffisant pour cette installation.

> Compter à peu près 2 heures pour réaliser l'installation complète du système

## Installation de la distribution Ubuntu 18.04 sur clé USB ou SD Card

Récupérer l'image "Ubuntu 18.04 - Raspberry Pi 3 (64-bit ARM) preinstalled server image" depuis http://cdimage.ubuntu.com/releases/18.04/release/

Ecrire l'image sur une clé USB (version B+) ou une carte SD (version B ou B+)

**sous linux**

```bash
xzcat ubuntu-18.04.2-preinstalled-server-arm64+raspi3.img.xz | sudo dd bs=4M of=/dev/sdb
```

**sous mac os**

```bash
brew install xz
diskutil umountDisk /dev/disk2
xzcat ubuntu-18.04.2-preinstalled-server-arm64+raspi3.img.xz | sudo dd bs=4m of=/dev/rdisk2
```

## Modification du boot - Uniquement si installation sur clé USB

Monter la clé USB sur votre ordinateur et éditez le fichier cmdline.txt sur la partition "system-boot" en remplaçant son contenu par le suivant :

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

Sauvegarder le contenu du fichier "/etc/flash-kernel/bootscript/bootscr.rpi3" sur la partition "writable" dans un fichier ".bak" puis le remplacer par le contenu du fichier créer précédement.

Recompiler l'image de boot :

```shell
$ sudo apt-get install u-boot-tools
$ mkimage -A arm -O linux -T script -C none -n boot.scr -d boot.scr.txt boot.scr
```

Remplacez le fichier "boot.scr" sur la partition "system-boot".

Ajouter une ligne au fichier "/etc/default/raspi3-firmware":

```
ROOTPART=LABEL=writable
```

## Premier démarrage et mise à jour du système

Déconnecter tous les périphériques (LIDAR, périphérique I2S, SPI, USB, ...).

Brancher la clé USB ou insérer la carte SD dans le Raspberry Pi et l'allumer. 

En version USB, si le Raspberry ne réussi pas à booter, formater une carte SD en FAT32 et copiez dessus le contenu de la partition "system-boot".

Se connecter au raspberry :
* câble ethernet, écran via le port HDMI et un clavier 
* directement via éthernet en SSH

L'identifiant d'accès par défaut est ubuntu / "pas de mot de passe" ou ubuntu / ubuntu

Mise à jour du système et redémarrage :

```bash
sudo apt-get update
sudo apt-get upgrade
```

## Installation de NetworkManager et configuration du WIFI

Installation du network-manager :

```bash
sudo apt install network-manager
sudo systemctl start NetworkManager
sudo systemctl enable NetworkManager
```

Ajouter votre connection WIFI et modifier le nom de la machine ("stardust_r*X*") :

```bash
sudo nmtui
```

La suite de l'installation peut être réalisée à distance, via SSH.


## Optimisation de l'espace disque et de la vitesse de Boot

Suppression de tous les paquets dont l'on peut se passer afin de minimiser l'utilisation de l'espace disque et de la mémoire vive :

```bash
sudo apt remove cloud-init open-iscsi unattended-upgrades apparmor plymouth apport snapd
sudo apt autoclean
```

Suppression de tous les services dont l'on peut se passer afin d'accélérer le démarrage du robot et minimiser l'utilisation de la mémoire vive :

```bash
sudo systemctl mask systemd-networkd-wait-online.service 
sudo systemctl mask systemd-networkd-wait-online.service
sudo systemctl mask apt-daily.service apt-daily-upgrade.service
sudo systemctl mask ModemManager.service
sudo systemctl mask pppd-dns.service
sudo systemctl mask accounts-daemon.service
sudo systemctl mask avahi-daemon.service
sudo systemctl mask plymouth.service
sudo systemctl mask apport.service
```

Puis, ignorer l'attente de la connexion filaire (eth0) :

```bash
sudo vi /lib/systemd/system/systemd-networkd-wait-online.service
```

Modifier la ligne ExecStart de la manière suivante :

```
...
[Service]
Type=oneshot
ExecStart=/lib/systemd/systemd-networkd-wait-online --ignore=eth0
RemainAfterExit=yes
...
```

Pour en savoir plus : https://www.linux.com/learn/cleaning-your-linux-startup-process

## Désactivation des mises à jour automatiques

Les mises à jour automatiques en arrière plan peuvent entre en conflit avec le programme du robot, elles doivent donc être désactivées.

Editer le fichier "/etc/apt/apt.conf.d/20auto-upgrades" et remplacez les lignes :

```
APT::Periodic::Update-Package-Lists "1";
APT::Periodic::Unattended-Upgrade "1";
```

par

```
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";
```

## Optimisation de la vitesse de connexion en SSH

Suppression de tous les message à la connexion (motd):

```bash
sudo chmod -x /etc/update-motd.d/*
```

Redémarrer :

```bash
sudo reboot
```

L'accès au SSH après la commande reboot doit être désormais possible en moins de 40 secondes.
Pour voir le temps pris au démarrage par chaque service :

```bash
systemd-analyze blame
```

Pour voir la mémoire utilisée (normalement autour de 80 Mo après les optimisations) :

```bash
free
```

## Installation de ROS

Source : http://wiki.ros.org/Installation/UbuntuARM
L'installation de ROS est conséquente (Environ 1 Go) et dure plusieurs minutes

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install -y ros-melodic-ros-base git python-wstool vim
sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Installation des dépendances du code source du robot

Le code source du robot est devenu trop conséquent pour être compilé directement sur le raspberry pi, ou alors en étant patient et en utilisant un espace SWAP d'au moins 1 Go. La procédure illustre donc comment déployer le code via cross compilation depuis une autre machine Ubuntu. Mais avant celà, il faut installer les dépendances.

Récupérération du code source pour calcul et installation des dépendances :

```shell
cd ~
git clone https://github.com/julienbayle/stardust.git
~/stardust/scripts/update.sh --no-build
```

Puis déployer le code source depuis un poste distant et installer celui-ci pour lancement automatique au démarrage :

```shell
ssh-copy-id <robot_user>@<robot-hostname>
deploy-rpi.sh -remote-hostname=<robot-hostname> --version=stardust_v1 --robot-name=r1 --build --install-on-startup
```

Ou simplement déployer le code sans installation au démarrage :

```shell
ssh-copy-id <robot_user>@<robot-hostname>
deploy-rpi.sh -remote-hostname=<robot-hostname> --version=stardust_v1 --robot-name=r1 --build
```

[Détails et explications sur la cross compilation et la commande deploy-rpi.sh](cross_compilation_raspberry.md).


### Lancer le programme (premier démarrage)

Tester que le projet démarre (remplacer r1 par r2 pour le robot secondaire) :
	
```bash
~/stardust_v1/scripts/start.sh rX
```

Pour référence (ne pas utiliser), voici la procédure pour compiler le code source sur le raspberry pi directement :
	
```bash
sudo fallocate -l 1024m /1014m.swap
sudo chmod 600 /1024m.swap 
sudo mkswap /1024m.swap 
sudo swapon /1024m.swap
~/stardust/scripts/update.sh
sudo swapoff /1024m.swap
```

### Optionnel - Activation de l'I2C et du SPI

Si le robot utilise des périphériques I2C ou SPI, appliquer cette étape avant de les rebrancher.

Editez le fichier **/boot/firmware/config.txt** et vérifier que les lignes suivantes sont bien présentes (sinon, les ajouter) :

```
dtparam=i2c_arm=on
dtparam=spi=on
```

Installez les librairies de developpement:

```bash
sudo apt-get install i2c-tools libi2c-dev
sudo usermod -a -G i2c <user_name>
```

Se déconnecter (exit) puis se reconnecter afin de prendre en compte le nouveau groupe.

Exemple : Si une IMU est connectée au robot via I2C, il est possible de vérifier qu'elle est bien disponible sur le port I2C et que celui-ci marche bien :

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

### Optionnel - Support du XV11 LIDAR (UART)

Si le robot utilise un lidar issu du robot Neato XV 11, appliquer cette étape avant de le rebrancher.

La procédure est documentée sur la [page officielle du paquet ROS xv_11_lidar_raspberry](https://github.com/julienbayle/xv_11_lidar_raspberry)

### Optionnel - Support des pavés LED

Si le robot utilise des pavés LED lié au SPI du raspberry, appliquer cette étape avant de les rebrancher.

Ajouter la librairie nécessaire (pas de paquet rosdep de disponible) :

```bash
sudo apt install python-pip
sudo pip install luma.led_matrix
```

Puis, il faut ajouter les droits d'utilisation le port SPI pour l'utilisateur courrant :

```bash
sudo groupadd --system spi
sudo adduser <user_name> spi
sudo  vim /etc/udev/rules.d/90-spi.rules
```

Contenu à écrire dans le fichier *90-spi.rules* :
```
SUBSYSTEM=="spidev", GROUP="spi"
```

Puis, redémarrer le système pour prise en compte des modifications

## Optionnel - Ajout d'une manette

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

Pour démarrer ROS manuellement (En tâche de fond) :

```bash
~/stardust/scripts/start.sh rX
```

Pour arrêter ROS manuellement :

```bash
~/stardust/scripts/stop.sh
```

Pour mettre à jour le code ROS et les dépendances :

```bash
~/stardust/scripts/update.sh --no-build
```

Activation du démarrage automatique de ROS au démarrage du Raspberry PI (on se place dans le dossier script depuis un PC distant) :

```bash
./deploy-rpi.sh --robot-name=rX --install-on-startup
```

## Accès au robot depuis un PC distant

Exemple pour lancer rviz depuis un poste distant (ROS doit être lancé sur le robot via le script ci-dessus et le robot et l'ordinateur doivent être sur le même réseau).

Ajouter le nom de la machine du robot dans le fichier /etc/hosts puis :
```bash
source ~/stardust/scripts/source-pc-slave.sh <robot_hostname>
rosrun rviz rviz
```

## Sauvegarde de la SD Card

Afin d'éviter d'avoir à refaire toute l'installation en cas d'erreur, il est préférable de sauvegarder le media utilisé en fin de procédure. La durée de la sauvegarde dépend de la taille du media (sd card de 64 Go = 1h par exemple). La sauvegarde ne peut être restaurée que sur un media de taille au moins équivalente.

L'utilisation de la compression xz de niveau 9 permet d'avoir une image de seulement 1,3 Go en fin d'installation.

```bash
diskutil umountDisk /dev/disk2
sudo dd if=/dev/rdisk2 bs=1m | xz -9 > stardust_r2_sd_card_bs_1m_64go.img.xz
```

La restauration utilise la même procédure à l'envers :

```bash
diskutil umountDisk /dev/disk2
xzcat stardust_r2_sd_card_bs_1m_64go.img.xz | sudo dd bs=1m of=/dev/rdisk2
```

