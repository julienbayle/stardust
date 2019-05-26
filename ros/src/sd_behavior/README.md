# Behavior tree

Ce module orchestre le fonctionnement du robot. L'implémentation est basée sur des arbres de comportement (behavior tree).

La prise en main de ce module n'est pas évidente. Aussi, il est important de lire et comprendre toute la [documentation](https://www.behaviortree.dev) de la bibliothèque BehaviorTree.CPP avant de commencer à mettre à jour le behavior tree du robot.

Pour aller plus loin, il est possible de lire quelques classes du coeur de la bibliothèque. Le [code est hébergé sur GitHub](https://github.com/BehaviorTree/BehaviorTree.CPP).

## Groot

[Groot](https://github.com/BehaviorTree/Groot) est un outil graphique pour mettre à jour, monitorer en temps réel et comprendre l'exécution de l'arbre de comportement d'un robot.

Installation :

```bash
sudo apt-get install qtbase5-dev qtbase5-dev libqt5svg5-dev
git clone https://github.com/BehaviorTree/Groot.git
cd Groot
git submodule update --init --recursive
```

Ensuite, il faut patcher le code car malheureusement, il y a un bug qui fait planter l'ouverture des fichiers de log :

```bash
vi depend/BehaviorTree.CPP/include/behaviortree_cpp/flatbuffers/flatbuffers.h
```

On commente dans ce fichier la ligne 1988 :
```
//FLATBUFFERS_ASSERT(size_ < FLATBUFFERS_MAX_BUFFER_SIZE);
```

Puis on compile
```bash
mkdir build; cd build
cmake ..
make
```

Groot est capricieux, il plante très souvent. Mais il faut lui donner sa chance ! Ou prendre son mal en patience....

Lancer Groot :
```bash
chmod +x Groot
./Groot
```

Groot permet : 
 - de monitorer l'exécution en temps réel
 - de revisiter le dernier log d'exécutions (stockés par défaut sous .ros/bt_trace.fbl)
 - modifier un arbre (fichier xml du dossier config de ce module)

## Démarrer l'arbre de comportement sur son poste local

Démarrer le robot puis lancer dans un terminal sur son poste local :

```bash
cd stardust/ros
catkin_make
source devel/setup.sh
source ../scripts/source-pc-slave.sh stardust_r1
roslaunch sd_behavior sd_behavior.launch robot_ns:=r1
```

## Démarer l'arbre de tests

Un arbre de test a été mis au point afin de tester plus rapidement le bon fonctionnement des différents noeuds.

Pour démarrer le graphe de test :

```bash
cd stardust/ros
catkin_make
source devel/setup.sh
roslaunch sd_behavior sd_behavior_test.launch
```

Pour simuler que tous les capteurs du robot R1 sont actif (et en conséquence, changer de camp) :

```bash
rostopic pub /r1/pilo/switches std_msgs/UInt32 "data: 1023" --rate 10
```

Pour voir l'affichage du score en temps réel :

```bash
rostopic echo /r1/lcd/line1
```

## Convention pour les positions du robots sur le terrain (MoveNodes)

Orientation du terrain : 
 - x dans le sens de la largeur, positif en direction des balances
 - y dans le sens de la longueur, positif en direction du camp jaune

Le centre du terrain est considéré comme l'origine du repère

L'angle est égal à 0 face aux balances
L'angle est positif en tournant dans le sens inverse des aiguilles d'une montre

## Edition de l'arbre à chaud

Il est possible de modifier l'arbre à chaud sur le robot. Le robot recharge alors l'arbre automatiquement.
Par contre, le "monitoring temps réel via Groot" n'est alors plus mis à jour après le rechargement.

Exemple :

```bash
vi stardust_20190522/install/share/sd_behavior/config/r2_bt.xml
```

## Analyse des logs sur le poste local

Récupération des logs sur le robot :

```bash
rsync ubuntu@stardust_r2:/home/ubuntu/.ros/bt_trace.fbl .
```

Ouvrir Groot en mode "Log Replay" et ouvrir le fichier récupéré/