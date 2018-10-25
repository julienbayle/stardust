# Développer en local

Le code du robot est en [ROS Kinetic](http://wiki.ros.org/kinetic/Installation) (Kinetic étant la version stable actuelle).

Afin d'avoir un environnement simple d'emploi, l'installation d'Ubuntu version 16 et uniquement cette version est conseillé. Testé et approuvé, si l'on utilise d'autres OS ça marche. Par contre, il faut aimer les forums en ligne, les moteurs de recherche pour trouver des réponses à chaque fois que l'on a une diffculté. En bref, on revient sur Ubuntu 16 et comme celà, on code en paix.

Trois techniques :
- Installer Ubuntu 16 sur son ordinateur
- Installer Ubuntu 16 sur W10 via le store
- Installer Ubuntu 16 via une machine viruelle

Chacun est libre de l'installer comme il le préfère et comme il le sent.

## Ubuntu 16 avec windows 10 sans effort

Pas besoin d'une licence windows 10, windows 10 marche désormais sans licence !

Mettre à jour son Windows 10 pour être dans une version supérieure à la 1703.

> Menu windows > Paramètres > Mises à jour et sécurité > Information sur la version du système d'exploitation

Installer le sous-système pour Linux

> Menu windows > Powershell (clic droit "Exécuter en tant qu'administrateur")

Puis entrer :

```
Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux
```

Redémarrer pour finaliser l'installation.

Ouvrir le windows store

> Menu windows > Microsoft store

Rechercher "Ubuntu" et installer "Ubuntu 16.04 LTS"

Un raccourci apparait (logo rond et orange) dans votre barre des tâches windows. C'est prêt ! Login et mot de passe identiques à ceux de la session Windows.

Afin de pouvoir lancer les applications graphiques depuis le sous-système ubuntu, il faut installer un serveur X sous windows 10. XMing n'a jamais marché en local mais VCXSrv fonctionne à merveille.

Télécharger et installer https://sourceforge.net/projects/vcxsrv/ (Ne pas oublier de bien autoriser le logiciel en cochant toute les cases lorsque Windows demande confirmant des accès réseaux autoriser au logiciel)

Ouvrir la console Ubuntu depuis Windows et préparer l'utilisation du serveur X :

```bash
sudo su
dbus-uuidgen > /var/lib/dbus/machine-id
exit
echo "export DISPLAY=:0" >> ~/.bashrc
souce ~/.bashrc
```

## Installer ROS

Ouvrir une console Ubuntu 16 :

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

## Installer le code source du projet

Récupérer les sources depuis github :

```bash
cd ~
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

## Démarer l'environnement de simulation

```bash
roslaunch sd_main sim.launch
```

## Divers

Quelques conseils pour personnaliser son Ubuntu (Tout ce qui est dans cette partie est optionnel)

### VIM autocomplete pour ROS

Installation (VIM autocomplete and VIM-Plug)

```bash
sudo apt-get install vim-nox-py2
curl -fLo ~/.vim/autoload/plug.vim --create-dirs https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim
```

Editer VIMRC (.vimrc) et ajouter :

```bash
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
	
Ouvir VIM et entrer ":PluginsInstall"

Pour finaliser l'installation de *YouCompleteMe*, il faut lancer une compilation manuellement :

```bash
cd .vim/plugged/YouCompleteMe/
./install.py
```

### Sublime text 3

Installation :

```bash
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
sudo apt-get install apt-transport-https
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
sudo apt-get update
sudo apt-get install sublime-text
```

Exécution :

```bash
/opt/sublime_text/sublime_text &
```