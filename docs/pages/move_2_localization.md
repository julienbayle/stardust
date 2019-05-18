# Déplacement - Article 2 - Localisation du robot

Ce second article sur le déplacement du robot traite de la localisation du robot dans l'espace. En effet, l'odométrie, calculée dans l'article précédent n'est pas un moyen suffisant pour réaliser un suivi de trajectoire précis dans le temps. La raison est que cette mesure dérive au cours du temps. Le premier cas est lorsque le robot est bloqué, les roues tournent mais le robot ne bouge pas. Une autre raison est la présence de glissements au niveau des roues, non mesurables, qui font que le robot avance moins vite que prévu.

L'objectif de cette étape est de permettre au robot de savoir où il se trouve dans le référentiel "map" (la carte du monde). Autrement dit, de contrer la dérive du robot liée à son odométrie. On va ainsi chercher à estimer la position du robot par d'autres moyens permettant une localisation absolue.

Pour se faire, la première étape est d'améliorer l'odométrie en la fusionnant avec les mesures de la centrale inertielle. L'avantage de cette méthode est que l'odométrie n'est pas précise pour mesurer les rotations, alors que la centrale inertielle l'est. On obtient ainsi une **odométrie filtrée** dont la qualité est supérieure.

La seconde étape est d'utiliser les mesures du LIDAR pour corriger le déplacement par rapport à une mesure du déplacement faite entre deux trames LIDAR ainsi que la comparaison de l'image LIDAR avec une carte théorique.

Le paquet **sd_localization** s'occupe de tout ces aspects. Son rôle est donc de fusionner odométrie, IMU et LIDAR pour obtenir la meilleure localisation possible du robot dans l'espace relativement au repère absolu map. 

Cet article évoque le réglage des paramètres des noeuds de ce paquet afin d'avoir en place tous les ingrédients de base pour la planification des trajectoires du robot et l'évitement d'obstacle, objet du prochain article.

## Fusion odométrie, IMU : Calcul de l'odométrie filtrée

Le paquet **sd_sensors** publie les données brutes de la centrale inertielle (ima/data_raw). Ces données brutes sont des mesures de vitesse bruitées et biaisées. Elles sont améliorées par un premier noeud (http://wiki.ros.org/imu_filter_madgwick) qui transforme ces mesures de vitesse en données complètes (orientation intégrée, débruitée et dé-baisée) et les publie (imu/data).

Pour valider le bon fonctionnement de ce paquet, il faut afficher les données du topic imu/data et valider que les données sont correctes.

Pour bien calculer les biais de la centrale inertielle, un service a été mis au point. Pour l'appeler il se connecter au Pi et entrer :

```bash
source install/setup.bash
rosservice call /r2/mpu6050/get_bias
```

Puis reporter les données dans la configuration de l'IMU du paquet **sd_sensors** 

Le second noeud fusionne les données IMU avec celle de l'odométrie afin de publier une odométrie filtrée (odom/filtered) et la transformation TF odom->base_link. Il s'agit en pratique d'un filtre *Unscented Kalman Filter* proposé dans le paquet [robot_localization]("http://docs.ros.org/melodic/api/robot_localization/html/index.html). La [documentation](http://docs.ros.org/melodic/api/robot_localization/html/state_estimation_nodes.html) précise comment régler les différents paramètres. 

Pour valider le bon fonctionnement de ce filtre, on balade le robot à la télécommande et on valide que le topic odom/filtered contient des données de qualité. Une autre technique est de visualiser le robot dans RVIZ dans la base rX/odom et valider que les déplacements sont logiques, surtout les rotations.


## AMCL : Estimation de la position absolu

Sans mesure absolue de la position, il serait impossible de recaler la dérive de la position mesurée ci-avant. Le LIDAR est donc utilisé pour fournir à une fréquence faible une donnée de localisation absolue basée sur l'analyse des mesures du LIDAR par rapport à une carte connue du monde. 

Le noeud AMCL du paquet **sd_localization** joue ce rôle. AMCL est un système de localisation probabiliste pour un robot 2D qui implémente l'algorithme "adaptive (or KLD-sampling) Monte Carlo localization". Il s'agit en pratique d'une approche basée sur un filtre particulaire qui cherche à déterminer la position du robot dans une carte connue.

Le noeud rX_map_server se charge d'émettre la carte connue sur le topic map. Cette carte est construite manuellement, il s'agit d'une simple image PNG associée à une fichier de description. Cette carte doit être celle du monde connue à la hauteur du laser (un obstacle non perceptible à la hauteur du LIDAR ne doit pas faire parti de cette carte).

Plusieurs cartes sont disponibles :

- table : la table de jeu
- office : le bureau de Julien

Par défaut, le robot démarre avec la table de jeu. Pour le modifier, il faut lancer le robot en précision la carte à charger :

```bash
source install/setup.bash
roslaunch sd_main r2.launch location:=office
```

Cet algorithme ne peut pas trouver seul la position du robot, il faut l'initialiser avec la position réelle au démarrage ou une position très approchées. Le topic rX/initialpose permet ceci. Le plus simple est d'émettre la position initiale via l'action de behavior tree, le message start avec l'équipe (voir paquet sd_behavior) ou encore via RVIZ.

Dans RVIS, il faut faire clic droit sur "2D pose estimate" puis régler le nom du topic. Ensuite, en se placant dans la frame "map" en tant que frame fixe, on peut idenquer au robot sa position.

Si tout se passe bien, l'algorithme AMCL commence alors à fonctionner correctement. Pour valider les réglages, on déplace le robot à la télécommande et on vérifie que la position reste bonne et que les mesures du LIDAR restent concordante avec la carte du monde. Il est également essentiel de valider que la charge CPU des calculs est soutenable pour le robot. En effet, certains réglage ne permettent pas un calcul temps réel des informations.

Malheureusement, ce filtre a plusieurs limites :

- il peut "sauter", c'est à dire se perdre temporairement puis reconverger
- se perdre totalement et ne plus fonctionner
- se perdre puis reprendre à une position erronée et sauter ainsi de position erronée en position erronée
 
 Il est donc essentiel de bien le régler et ne pas permettre au robot d'aller plus vite que la vitesse à laquelle ce filtre peut "suivre et comprendre". Sinon, comportements bizarres garantis.
  
Deux améliorations sont possibles (optionnelles) :

- Filtrer les données d'AMCL via un autre filtre UKF qui se chargera alors d'émettre la TF map à sa place
- Associé AMCL à un algorithme de recalage automatique (snapmap par exemple) qui supervise AMCL et forcer une réinitialisation lorsque la qualité de la localisation passe en deça d'un certain critère. Au delà, ce noeud peut également publier en temps réel un topic permettant de mesurer la qualité de l'estimation de la pose du robot par AMCL.

## Des soucis, des outils pour aider

Voir les données des TF en temps réel :

```bash
source source-pc-slave.sh stardust_rX
rosrun tf tf_monitor
```

Voir l'arbre des TF en détail, ainsi qu'une analyse des retards et fréquences (génére un fichier PDF) : 

```bash
source source-pc-slave.sh stardust_rX
rosrun tf view_frames
```

Voir l'ensemble des éléments via RVIZ et une configuration adaptée (sd_description/rviz/rX.rviz) :

```bash
source source-pc-slave.sh stardust_rX
rosrun rviz rviz &
```