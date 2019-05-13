# Déplacement - Article 2 - Localisation du robot

Ce second article sur le déplacement du robot traite de la localisation du robot dans l'espace. En effet, l'odométrie, calculée dans l'article précédent n'est pas un moyen suffisant pour se repérer dans l'espace. La raison est que cette mesure dérive au cours du temps. Le premier cas est lorsque le robot est bloqué, les roues tournent mais le robot ne bouge pas. Une autre raison est le présence de glissements au niveau des roues, non mesurables, qui font que le robot avance moins vite que prévu.

L'objectif de cette étape est de permettre au robot de savoir où il se trouve dans le référentiel le plus général, appelé "map". Ou autrement dit, de calculer la dérive du robot relativement à son odométrie, c'est à dire la transformation entre le référentiel odom (où le robot pense être en s'appuyant uniquement sur l'intégration temporelle de l'odométrie) et le référentiel map (où le robot est réellement dans l'espace).

Pour se faire, la première étape est d'améliorer l'odométrie en la fusionnant avec les mesures de la centrale inertielle. L'avantage de cette méthode est que l'odométrie n'est pas précise pour mesurer les rotations, alors que la centrale inertielle elle oui. On obtient ainsi une **odométrie filtrée** dont la qualité est supérieure.

La seconde étape est d'utiliser les mesures du LIDAR pour corriger le déplacement par rapport à une mesure du déplacement faite entre deux trames LIDAR ainsi que la comparaison de l'image LIDAR avec une carte théorique.

Le paquet **sd_localization** s'occupe de tout ces aspects. Son rôle est donc de fusionner odométrie, IMU et LIDAR pour obtenir la meilleure localisation possible du robot dans l'espace relativement au repère absolu map. 

Cet article évoque le réglage des paramètres des noeuds de ce paquet afin d'avoir en place tous les ingrédients de base pour la planification des trajectoires du robot et l'évitement d'obstacle, objet du prochain article.

## Fusion odométrie, IMU : Calcul de l'odométrie filtrée

Le paquet **sd_sensors** publie les données brutes de la centrale inertielle (ima/data_raw). Ces données brutes sont des mesures de vitesse bruitées et biaisées. Elles sont améliorées par un premier noeud (http://wiki.ros.org/imu_filter_madgwick) qui transforme ces mesures de vitesse en données complètes (orientation intégrée, débruitée et dé-baisée) et les publie (imu/data).

Le second noeud fusionne les données IMU avec celle de l'odométrie afin de publier une odométrie filtrée (odom/filtered) et la transformation TF odom->base_link. Il s'agit en pratique d'un filtre *Unscented Kalman Filter* proposé dans le paquet [robot_localization]("http://docs.ros.org/melodic/api/robot_localization/html/index.html).

http://docs.ros.org/melodic/api/robot_localization/html/state_estimation_nodes.html

Pour vérifier que ces noeuds fonctionnent correctement, il faut vérifier que les données émises sont cohérentes (en bougeant le robot à la télécommande par exemple) et vérifier que les fréquences sont stables et cohérentes avec les paramétrages :


Vérification que tout fonctionne correctement :
hz odom, imu_data, imu

## AMCL : Calcul de la transformation map->odom

Sans mesure absolue de la position, il serait impossible de recaler la dérive de la position mesurée ci-avant. Le LIDAR est donc utilisé pour fournir à une fréquence faible une données de localisation absolue basée sur l'analyse des mesures du LIDAR par rapport à une carte connue. 

Le noeud AMCL du paquet **sd_localization** joue ce rôle. AMCL est un système de localisation probabiliste pour un robot 2D qui implémente l'algorithme "adaptive (or KLD-sampling) Monte Carlo localization". Il s'agit en pratique d'une approche basée sur un filtre particulaire qui cherche à déterminer la position du robot dans une carte connue.

Le noeud r*_map_server se charge d'émettre la carte connue sur le topic map. Cette carte est construite manuellement, il s'agit d'une simple image PNG associée à une fichier de description.

Le topic initialpose permet de fournir à l'algorithme une estimation de la position du robot via une source externe (c'est le cas de la position de démarrage du robot par exemple).

Malheureusement, ce filtre ne peut aller plus vite que la fréquence du LIDAR et la mesure LIDAR et le calcul AMCL introduise une latence importante dans la mesure. Aussi, quand ce noeud trouve la position, c'est souvent avec plusieurs centaines de millisecondes de retard avec la réalité. Pour diminuer l'influence de ce retard et augmenter la fréquence de la prédiction de la TF map, un autre noeud est ajoutée.

Ce dernier noeud permet de fournir une TF map à une fréquence régulière. Il s'agit d'un second filtre *Unscented Kalman Filter*. Si AMCL ne trouve pas la position du robot pendant un certain temps, ce sont les données odom (vitesse et non position) et IMU qui sont utilisée pendant cette durée pour estimer le TF map->base_link

Enfin, un dernier noeud **snapmap** permet de superviser AMCL et forcer une réinitialisation lorsque la qualité de la localisation passe en deça d'un certain critère. Au delà, ce noeud peut également publier en temps réel un topic permettant de mesurer la qualité de l'estimation de la pose du robot par AMCL :

During operation amcl estimates the transformation of the base frame (~base_frame_id) in respect to the global frame (~global_frame_id) but it only publishes the transform between the global frame and the odometry frame (~odom_frame_id). Essentially, this transform accounts for the drift that occurs using Dead Reckoning. The published transforms are future dated. 

rosrun tf tf_monitor
