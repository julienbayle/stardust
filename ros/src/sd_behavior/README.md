# Behavior tree

Ce module propose une implémentation en behavior tree du comportement du robot.

Il est important de lire et comprendre toute la documentation suivante avant de commencer à mettre à jour le behavior tree du robot (tant le code que le graphe de comportement) :

- https://www.behaviortree.dev
- https://github.com/BehaviorTree/BehaviorTree.CPP
- https://github.com/BehaviorTree/Groot

Pour lancer le behavior tree sans démarrer tout le robot :

```bash
roslaunch sd_behavior sd_behavior.launch robot_ns:=r1
```

Pour démarrer le graphe de test (graphe permettant de valider le comportement de tous les noeuds) :

```bash
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