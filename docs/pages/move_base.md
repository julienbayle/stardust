
Move base :
http://wiki.ros.org/move_base?distro=melodic

goal => make a plan => Fail => recovery_behaviors => Success or fail
http://wiki.ros.org/clear_costmap_recovery

2 cartes :
http://wiki.ros.org/costmap_2d?distro=melodic
=> En pratique, pluseiurs couche par carte
While each cell in the costmap can have one of 255 different cost values

2 planner :
local
global

1 ordonnanceur : move_base

Le global planner calcule une trajectoire (suite de positions) afin de permettre au robot d'aller de la pose de départ à la pose d'arrivée en tenant compte des obstacles connus
Attention : le camp adverse doit être vu comme une zone léthale (car interdite) même si il ne s'agit pas.
Carte des zones interdites à ajouter.
+ init appel au service clear_costmaps (on reset les obstacles)

Cette trajectoire n'a pas de réalité temporelle et peut être remise en compte par la découverte d'obstacle sur le chemin, ignorés lors du calcul.

GlobalPlanner/plan => Global plan

http://wiki.ros.org/costmap_2d?distro=melodic

Local Planner :

Son rôle est d'éxécuter la trajectoire localement, cette fois ci en tenant compte de la réalité du robot (contrainte de taille, de vitesse et accélération limite, des imprécisions d'exécution de la trajectoire...)
et des obstacles imprévus

DWA : Dynamic window approch
- A chaque instant, recherche le couple (v,w) tel que :
 - le robot ne percutte pas d'obstacle (couple admissible) à t+n 
 - le couple à t+1 est dans un fenêtre autour du couple t (évolution continue des vitesses dans le temps - respect des accélérations max)
 - le couple choisi est admissible et réalisable et le robot avance le plus vite possible dans la direction prévue par la trajectoire globale

http://ais.informatik.uni-freiburg.de/teaching/ss10/robotics/slides/16-pathplanning.pdf

C'est un algorithme rapide et performant mais ne garantissant pas un plan d'exécution optimal.
Le robot a en effet du mal à suivre des trajectoires dans des environnements très contraints (passage de la largeur du robot entre des obstacles ou navigation dans des environnements très encombrés)

~<name>/global_plan (nav_msgs/Path)
The portion of the global plan that the local planner is currently attempting to follow. Used primarily for visualization purposes.
~<name>/local_plan (nav_msgs/Path)
The local plan or trajectory that scored the highest on the last cycle. Used primarily for visualization purposes.
~<name>/cost_cloud (sensor_msgs/PointCloud2)
The cost grid used for planning. Used for visualization purposes. See the publish_cost_grid_pc parameter for enabling/disabling this visualization. New in navigation 1.4.0

