# Développer en local

Le code du robot est en [ROS Kinetic](http://wiki.ros.org/kinetic/Installation) (Kinetic étant la version stable actuelle).

Afin d'avoir un environnement simple d'emploi, l'installation d'Ubuntu version 16 et uniquement cette version est conseillé. Testé et approuvé, si l'on utilise d'autres OS ça peut marcher. Par contre, il faut aimer les forums en ligne, les moteurs de recherche pour trouver des réponses à chaque fois que l'on a une diffculté. En bref, on revient sur Ubuntu 16 et comme celà, on code en paix.

Trois techniques :
- Installer Ubuntu 16 via une machine virtuelle (le plus simple, le VMDK est prêt)
- Installer Ubuntu 16 sur son ordinateur puis ROS (plus rapide)
- Installer Ubuntu 16 sur W10 via le store puis ROS (Si votre carte graphique est supporté sur Ubuntu 16)

Chacun est libre de l'installer comme il le préfère et comme il le sent.

## Ubuntu 16 via une machine virtuelle

Une machine virtuelle VMWare a été préparée pour le club (Niveau de compatibilité VMWARE 14).

Elle intègre :
 - Ubuntu (installation de base sans rien de spécifique)
 - VMWare tools (version disponible dans les dépôts Ubuntu)
 - ROS et le projet (version à la date de création de la machine virtuelle, installé selon la procédure décrite ci-après)

Pour la récupérer, envoyer un email à Julien avant une réunion du club.

Lancer VMWare et importer la machine virtuelle.

Login : stardust
Password : stardust

Le code est installé dans le /home de l'utilisateur.

Si Gazebo crashe, désactiver l'accélération 3D dans les propriétés de la machine sous vmware. En effet, les derniers VMWare Tools ne peuvent pas être installés pour Ubuntu 16, ce qui pose souci sur certaines machines récentes.

Vous pouvez également réaliser votre propre VM, rien de particulier à prendre en compte à part la version d'Ubuntu.

## Installation Ubuntu 16

### Ubuntu 16 sur son ordinateur

Aller sur http://releases.ubuntu.com/16.04.5/ et télécharger l'ISO

Le copier sur une clé USB et redémarrer. Puis suivre les écrans d'installation (cocher "installer les mises à jour lors de l'installation" et "installation des pilotes tiers").

### Ubuntu 16 avec windows 10

**Attention : N'utiler cette procédure qu'après avoir vérifier que les pilotes de votre cartes graphiques sont disponibles et supportés pour Ubuntu 16**

Mettre à jour son Windows 10 pour être dans une version supérieure à la 1703.

> Menu windows > Paramètres > Mises à jour et sécurité > Information sur la version du système d'exploitation

Si version inférieur à 1703, installer les dernières mises à jour.

Installer le sous-système pour Linux :

> Menu windows > Powershell (clic droit "Exécuter en tant qu'administrateur")

Puis entrer :

```
Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux
```

Redémarrer pour finaliser l'installation.

Ouvrir le windows store

> Menu windows > Microsoft store

Rechercher "Ubuntu" et installer "Ubuntu 16.04 LTS"

Un raccourci apparait (logo rond et orange) dans votre barre des tâches windows. C'est prêt ! (Login et mot de passe identiques à ceux de la session Windows)

Afin de pouvoir lancer les applications graphiques depuis le sous-système ubuntu, il faut installer un serveur X sous windows 10. XMing n'a jamais marché en local mais VCXSrv fonctionne à merveille pour moi.

Télécharger et installer https://sourceforge.net/projects/vcxsrv/ (Ne pas oublier de bien autoriser le logiciel en cochant toute les cases lorsque Windows demande confirmant des accès réseaux autoriser au logiciel)

Ouvrir la console Ubuntu depuis Windows et préparer l'utilisation du serveur X :

```bash
sudo su
dbus-uuidgen > /var/lib/dbus/machine-id
exit
echo "export DISPLAY=:0" >> ~/.bashrc
souce ~/.bashrc
```

Installer les pilotes de la carte graphique (selon votre carte graphique).



## Installation de ROS et du projet

### Installation de ROS

Avant d'installer ROS, mettre à jour votre système :

```bash
sudo apt-get update
sudo apt-get upgrade
```

Ouvrir une console Ubuntu 16 :

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install \
	ros-kinetic-desktop-full \
	ros-kinetic-gazebo-plugins \
	ros-kinetic-hector-gazebo-plugins \
	ros-kinetic-gazebo-ros-control
sudo apt install python-wstool mesa-utils
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Installation du code source du projet

Récupérer les sources depuis github :

```bash
cd ~
sudo apt-get install git
git clone https://github.com/julienbayle/stardust
```

Récupérer les dépendances (Pour information, elles sont listées dans le fichier .rosinstall du dossier ros/src) :

```bash
cd ~/stardust/ros/src
wstool update
```

Récupérer les dépendances des paquets (Chaque paquet ROS précise ses dépendances dans un fichier package.xml) :

```bash
cd ~/stardust/ros/
rosdep install --from-paths src --ignore-src -r -y
```

Compiler les sources et mettre à jour les raccourcis et l'autocomplétion bash pour le projet :

```bash
cd ~/stardust/ros/
catkin_make
source devel/setup.bash
```

## Démarage de l'environnement de simulation

```bash
roslaunch sd_main sim.launch
```

## Mettre à jour le code ROS

```bash
cd ~/stardust/ros/src
wstool update
cd ~/stardust/ros/
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```
