# Installation et mise à jour des cartes arduino

Les cartes arduino communiquent avec le raspberry via [ros_serial](http://wiki.ros.org/rosserial). Elles sont toutes connectées via un port série sur le Raspberry Pi.

L'outil [PlatformIO](https://platformio.org) est utilisée pour cette partie du projet.

## Préparation du Raspberry pour pouvoir compiler et mettre à jour les cartes

Les étapes de ce chapitre ne sont nécessaires que pour l'installation initiale du Raspberry et si l'on souhaite pouvoir mettre à jour les cartes depuis celui-ci.

Afin de pouvoir compiler et téléverser les codes Arduino, l'outil PlatformIO doit être installé (Uniquement les Shell commands, pas l'IDE) :

```bash
sudo pip install -U platformio
```

Les codes des cartes arduino dépendent de la librairie de ROS Serial version 0.8. Malheurement, cette librairie n'est pas encore disponible dans les dépendances de PlatformIO en version 0.8, il est donc nécessaire de la recompiler manuellement.

```bash
cd ~/stardust/arduino/lib
rosrun rosserial_arduino make_libraries.py .
```

## Compilation des firmwares arduino

Pour compiler tous les firmaware du projet, il suffit de taper la commande suivante :

```bash
cd ~/stardust/arduino
platformio run
```

Lors de la première exécution, la commande installe automatiquement tous les outils, librairies et dépendances nécessaires au projet.

## Mettre à jour le firmware d'une carte arduino

Exemple pour la carte Uno du second robot :

```bash
cd ~/stardust/arduino
platformio run -e r2-uno -t upload
```

En cas d'erreur du type : 
> /home/ubuntu/.platformio/packages/tool-avrdude/avrdude: line 6: /home/ubuntu/.platformio/packages/tool-avrdude/avrdude_bin: No such file or directory

Simplement faire un lien direct vers avrdude :
```bash
cd /home/ubuntu/.platformio/packages/tool-avrdude
mv avrdude_bin avrdude_bin_old
ln -s /usr/bin/avrdude avrdude_bin
```

> La liste des environnements est décrit dans le fichier [platformio.ini](https://github.com/julienbayle/stardust/blob/master/arduino/platformio.ini)

## Vérifier la bonne exécution du firmware depuis le raspberry pi

En supposant que la carte est accessible sur */dev/ttyACM0* :

```bash
roscore &
rosrun rosserial_python serial_node.py /dev/ttyACM0 &
rostopic list
```

Pour publier un message vers la carte, utiiser [rostopic pub](http://wiki.ros.org/fr/ROS/Tutorials/UnderstandingTopics). 

Pour lire les messages, utiliser [rostopic echo](http://wiki.ros.org/fr/ROS/Tutorials/UnderstandingTopics).
