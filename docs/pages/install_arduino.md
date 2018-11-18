# Installation et mise à jour des cartes arduino

Les cartes arduino communiquent avec le raspberry via [ros_serial](http://wiki.ros.org/rosserial). Elles sont toutes connectées via un port série sur le Raspberry Pi.

L'outil [PlatformIO](https://platformio.org) est utilisée pour cette partie du projet.

## Préparation du Raspberry pour pouvoir compiler et mettre à jour les cartes

Les étapes de ce chapitre ne sont nécessaires que pour l'installation initiale du Raspberry et si l'on souhaite pouvoir mettre à jour les cartes depuis celui-ci.

Afin de pouvoir compiler et téléverser les codes Arduino, l'outil PlatformIO doit être installé (Uniquement les Shell commands, pas l'IDE) :

```bash
sudo pip install -U platformio
```

Les codes des cartes arduinos dépendent de la librairie de ROS Serial. Le projet s'appuyant uniquement sur des messages standards, il n'est pas nécessaire de la recompiler. A la place, une dépendance vers une librairie ros_lib de PlatformIO est utilisée (voir fichier [platformio.ini](https://github.com/julienbayle/stardust/blob/master/arduino/platformio.ini)).

Pour référence, s'il fallait ajouter le support de messages spécifiques, il faudrait alors générer cette librairie à la main de la manière suivante :

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