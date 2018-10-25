Stratégie, objectif et exigences
=================================

Missions
--------

Le robot principal libère le palet goldenium puis récupére des palets pour les insérer dans l'accélérateur. En début de match, il lance l'expérience.

Le robot secondaire tri et pèse des palets. De manière accessoire, il est en capacité de lancer l’expérience si le robot principal est en panne pour un match.

Objectif
--------

Nombre de points cible total : 221 points / match

Nombre de points cible robot principal : 169 points

Détails robot principal :

* lance l'expérience : 40 points
* libérer et extrait le goldenium : 40 points
* 5 palets dans l'accélérateur : 50 points
* bonus évaluation de la performance : 39 points

Nombre de points cible robot secondaire : 52 points

Détails robot secondaire :

* classe :
** 2 palets rouges issus de la zone de démarrage : 10 points
** 2 autres palets trouvés (petit distributeur, table, …)  : 10 points

* pèse :
** 1 palet vert issu de la zone de démarrage : 8 points
** 1 palet rouge issu de la pente : 4 points
** 1 palet vert issu du petit distributeur : 8 points

* bonus évaluation de la performance : 12 points

Stratégie de match
------------------

Déroulement type d'un match (robot principal) :
1. On pose le robot sur le plateau et on l’allume
2. Le robot indique qu’il attend que l’on insère le cordon de démarrage
3. On insère le cordon de démarrage
4. Le robot se positionne automatiquement, détecte automatiquement son camp et confirme qu’il est prêt
5. Cordon de démarrage tiré, c’est parti !
6. 0 à 20 secondes : Le robot va cherche 3 palets sur le distributeur (l'expérience se lance)
7. 20 à 30 secondes : Le robot va face à l'accélérateur
8. 30 à 45 secondes : Le robot met 4 palet dans l'accélérateur
9. 45 à 60 secondes : Le robot libérer le goldenium
10. 60 à 98 secondes : Le robot va chercher d'autres palet et les insèrents dans l'accélérateur
11. 98 seconde : Le robot s’arrête et affiche son score

Déroulement type d'un match (robot secondaire) :
1. On pose le robot sur le plateau et on l’allume
2. Le robot indique qu’il attend que l’on insère le cordon de démarrage
3. On insère le cordon de démarrage
4. Le robot se positionne automatiquement, détecte automatiquement son camp et confirme qu’il est prêt
5. Cordon de démarrage tiré, c’est parti !
6. 0 à 5 secondes : Le robot détecte la couleur des palets à proximité (2 photos)
7. 5 à 20 secondes : Le robot attrape les deux palets rouges et les classent
8. 20 à 35 secondes : Le robot attrape le palet vert de la zone démarrage
9. 35 à 45 secondes : Le robot attrape le palet vert du petit distributeur
10. 45 à 55 secondes : Le monte la pente et pousse le palet présent sur la pente
11. 55 à 65 secondes : Le robot pousse les palets dans la balance et redescend
12. 65 à 98 secondes : Le robot recherche et classe d’autres palets présent sur la table
13. 98 seconde : Le robot s’arrête et affiche son score

Exigences
---------

La mission des robots est déclinée ici en exigences fonctionnelles

Une action de jeu permet de fixer un objectif au robot sur lequel il a autonomie de réalisation. Elle est susceptible de réussir ou échouer.

### Etre en capacité de se déplacer vers un objectif en évitant les collisions

L’exécution de la stratégie implique une distance totale parcourue d’environ 11 mètres. Les robots se déplaceront environ 75% du temps du match, soit une vitesse moyenne requise d’au moins 15cm/s.

Les robots évitent les collisions avec les autres robots et les éléments de jeu. Pour cela, ils sont dotés d’un LIDAR. Le LIDAR est placé afin de détecter les bordures mais pas les palets, soit à une hauteur comprise entre 4 et 5 cm. Le laser ne détectant pas les robots aux parois réfléchissantes, des capteurs complémentaires complètent le dispositif.

En permanence, les robots estiment leur position sur l’espace de jeu à l’aide de l’odométrie, complétée par un recalage issu des données du laser et eventuellement d'un gyromètre.

En permanence, les données du laser et des ultrasons sont traitées afin de détecter les obstacles dynamiques (autres robots). Ces données “obstacles” sont agrégées dans une carte en s’appuyant sur la position du robot recalée.

Une action de jeu permet de demander au robot de se rendre en position X,Y avec l’orientation theta avec une marge d’erreur configurable. Le robot génère des trajectoires basées sur la carte du plateau et tenant compte du repérage des obstacles dynamiques.

### Etre en capacité d’afficher son score et son état

Tout au long du match, l’estimation du score personnel s’affiche sur l'afficheur du robot. 

Un afficheur complémentaire affiche les données clés du fonctionnement du robot.

### Etre en capacité de lancer l'expérience

Les robots sont équipés d'un réflecteur permettant de lancer l'expérience lors de leurs premiers mouvements durant le match.

La position initiale des robots est telle que l'expérience capte les réflecteurs du robot principal et si celui-ci est absent, le réflecteur du robot secondaire.

### Avoir une autonomie de 4 matchs et 2h d’attente allumé

L'autonomie du robot est suffisante pour réaliser l’équivalent de 4 matchs entrecoupés d'une pause de 30 minutes.

### Etre en capacité de se positionner automatiquement et détecter son camp

Une action de jeu permet de demander au robot de se positionner seul et détecter seul son camp

### Etre télé-opérable

Les robots peuvent être déplacés à l’aide d'une télécommande (une par robot).

Le bras des robots est également téléopérable.

### Etre conscient de son état

Les robots auront les modes suivants “téléopéré”, 'calibration", “attente début match”, “prêt match”, “match”, “fin de match”

Au démarrage, le robot démarre en mode “téléopéré”. Dans ce mode, l'afficheur principal du robot affiche l’IP du robot.

Lorsque la tirette de démarrage est inséré, le robot passe automatiquement en mode “attente début match” et se positionne automatiquement (action de jeu : se positionner automatiquement et détecter son camp). A la fin de cette action, succès ou échec, le robot passe à l’état “prêt match”. En mode “prêt match”, l'afficheur du robot affiche la couleur du camps.

Lorsque la tirette de démarrage est retirée, si l’état est “prêt match”, le robot passe à l’état “match” et exécute le programme de match. En mode “match”, le robot est opéré à l'aide d'un arbre de comportement, le fonctionnement est donc prédictible et intelligent.

A la fin des 98 secondes ou en cas d’échec de la stratégie, le robot passe à l’état “fin de match”. Dans cet état, le robot affiche l’estimation de son score.

En cas d’appui sur le bouton d’arrêt d’urgence, le robot passe alors automatiquement en mode “fin de match”. 

A tout moment, l’appui sur le bouton “start” de la télécommande repasse le robot en mode “téléopéré”.

A tout moment, l’appui prolongé sur le bouton “start” de la télécommande repasse le robot en mode "calibration".

### Etre capable de calibrer ses capteurs et actionneurs

Une action de jeu "calibration" permet au robot de calibrer les éléments calibrables (biais IMU, ...)

### Respecter le règlement

Le robot principal fera un périmètre inférieur à 120 cm sur sa base (150 cm déployé).

Le robot secondaire fera un périmètre inférieur à 85 cm sur sa base (105 cm déployé).

Les robots font moins de 35 cm de hauteur 

Les robots ont un interrupteur/cordon de lancement de 1 mètre de long

Les robots ont un bouton d’arrêt d’urgence coupant les actionneurs du robot

Les robots disposent d’un support de balise amovible à la hauteur de 43 cm recouvert de velcro côté crochets et capable de supporter 300g

Un espace rectangulaire de 100 x 70 mm est laissé libre sur l’une des faces latérales de chaque robot 

Les robots sont équipés de LIDAR dont le laser est de classe 1 et peuvent le prouver

### Simulation 

Le fonctionnement des robots est simulable sur ordinateur avec une émulation de tous les capteurs et des actionneurs pour le déplacement des robots.

Des robots adverses factices et les éléments du plateau et de jeu sont simulés. 

Un moteur physique donnera du réalisme à cette simulation. 

### Autres exigences

Le raspberry doit être accessible afin de pouvoir changer la carte SD selon le besoin et y connecter un câble réseau.

Des fusibles limiteront les conséquences d’erreur de connexion

La tension de la batterie et la consommation du robot seront enregistrés en temps réel

Le robot devra pouvoir être éteint à la manette

Possibilité de recharger les batteries du robot sans avoir à le démonter

### Exigences spécifiques au robot principal

#### Etre en capacité de saisir des palets dans un distributeur

Le robot est capable de prendre des palets stockés sur un distributeur lorsque celui ci est placé devant le distributeur

#### Etre capable de pousser un palet dans l'accélérateur

Le robot peut pousser un palet présent sur la pente de l'accélérateur, qu'il transporte ou non des palets.

#### Etre capable de mettre des palets capturés dans l'accélérateur

Le robot est capable de libérer des palets dans l'accélérateur et de les pousser dans la pente

#### Etre capable de sortir le palet goldenium

Le robot est capable d'extraire le palet goldenium de son support sans pour autant être capable de le saisir.

### Exigences spécifiques au robot secondaire

#### Etre en capacité de détecter, positionner et capturer un palet

En permanence, le robot repére les palets à proximité et juge de leur position (sur la table, debout sur un distributeur) et leur couleur. Les palets détectés sont intégrés à la carte.

Une action de jeu permet de demander au robot de récupérer un palet au sol. Une action alternative demande au robot de capturer un palet d’une couleur donnée autour de lui.

Si une couleur de capture est configurée, les trajectoires évitent les palets détectés dont la couleur n’est pas conforme à la couleur de capture courante.

#### Etre en capacité de trier des palets 

Une action de jeu permet de demander au robot de trier les palets d’une couleur donnée. Le robot se rend alors dans la zone de tri correspond au camp actuel et à la couleur demandée.

#### Etre en capacité de peser des palets

Le robot est capable de monter la rampe (pente de 11°) en poussant jusqu’à 5 palets

Une action de jeu permet de demander au robot de peser les palets capturés. Le robot se dirigera vers le haut de la balance en tenant compte de son camp et pousse les palets dans la balance

