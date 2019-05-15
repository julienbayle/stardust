# Déplacement - Article 3 - Trajectoire et évitement d'obstacles

Les deux précédents articles ont permis de mettre au point un robot dont les mouvements son contrôlés et la position au cours du temps connue. Les bases pour générer et suivre des trajectoires en somme.

Le déplacement du robot est pris en charge par un seul noeud ros, il s'agit de [move_base](http://wiki.ros.org/move_base?distro=melodic).

Ce noeud se décompose en quatre éléments :

- Un planificateur de déplacement global (Global planner) associé à une carte globale des zones inconnues, libres ou encombrées.
- Un planificateur de déplacement local (Local Planner) associé à une carte locale des zones inconnues, libres ou encombrées.
- Un superviseur qui valide que le robot est bien en train d'avancer vers son objectif
- Une série de mécanisme que le superviseur peut activer si le robot a des difficultés (comportement de secours)

Le planificateur global calcule une trajectoire (suite de positions) afin de permettre au robot d'aller de la pose de départ à la pose d'arrivée en tenant compte des obstacles connus au moment de la planification dans la carte globale. Une zone peut être vide mais vue comme non libre, c'est par exemple le cas de la zone de départ du camp adverse. Ainsi, la carte sur laquelle se base se noeud est une carte du monde réel qui inclue également les territoires interdit (map_static_obstacle)

Cette trajectoire globale n'a pas de réalité temporelle et peut être remise en compte par la découverte d'obstacle sur le chemin.

Les cartes globale ou locale sont appelées des [costmaps](http://wiki.ros.org/costmap_2d?distro=melodic)

Le planificateur local a pour rôle d'éxécuter la trajectoire localement, cette fois ci en tenant compte de la réalité du robot (contrainte de taille, de vitesse et accélération limite, des imprécisions d'exécution de la trajectoire...) et des obstacles imprévus. Ce planificateur utilise un algorithme dit [DWA](http://ais.informatik.uni-freiburg.de/teaching/ss10/robotics/slides/16-pathplanning.pdf) pour *Dynamic window approach*. En pratique, il recherche à chaque instant, le couple (v,w) tel que :

 - le robot ne percutte pas d'obstacle (couple admissible)
 - le couple à t+1 est dans une fenêtre autour du couple t (évolution continue des vitesses dans le temps - respect des accélérations max)
 - le robot avance le plus vite possible dans la direction prévue par la trajectoire globale

C'est un algorithme rapide et performant mais ne garantissant pas un plan d'exécution optimal. Le robot a en effet du mal à suivre des trajectoires dans des environnements très contraint (passage de la

Le superviseur a pour rôle de détecter et traiter les échecs dans le cadre de l'atteinte d'un objectif :

1. Nouveau but : faire un plan global (si échec, échec)
2. Exécuter le plan - planificateur local (si échec, activation d'un mécanisme de secours)
3. Si échec après le mécanisme de secours, nouveau plan global (retour à l'étape 1) ou échec (nombre de tentatives limite atteint)

Pour valider le bon fonctionne de ce noeud, il faut demander des trajectoires avec RVIZ, analyser puis régler les paramètres des costmaps, des planner et du superviseur.

Ouvrir RVIZ et charger une configuration adaptée (sd_description/rviz/rX.rviz) :

```bash
source source-pc-slave.sh stardust_rX
rosrun rviz rviz &
```

> Attention, il faut aussi valider la charge CPU sur le robot durant l'exécution des trajectoires. Le robot peut très bien se comporter à l'arrêt ou à la télécommande mais ne pas avoir la puissance d'exécuter une trajectoire correctement. Un seul élément mal réglé peut prendre 100% d'un CPU. 

> Attention : il n'est pas possible de régler les trajectoires tant que le reste n'est pas "proche de la perfection", il faut alors reprendre les réglages de l'asservissement, de l'odométrie ou de la localization. Ce n'est pas cette couche qui solutionnera les défauts de celle d'en dessous... bien au contraire.