# Devenir une star !

Sois le bienvenu petit padawan de la robotique !

Tu trouveras sur cette page de quoi nourrir ton appétit de nouvelles connaissances pour améliorer ou mieux comprendre les robots de l'équipe.

## Arduino

> Les puces *arduino* sont utilisées sur les robots pour les fonctions bas niveau.

Savoir coder comme un dieu sur Arduino t'intéresse ?

[Démarrer avec Arduino](https://www.supinfo.com/articles/single/2721-bases-programmation-arduino)

[Aller plus loin et utiliser l'IDE des pro](https://platformio.org/platformio-ide)

## Raspberry pi

Le raspberry pi est une sorte de super arduino. Et oui, comme l'arduino, il possède des entrées-sorties appellé les GPIO (général purpose input output).

> Le raspberry pi est utilisé comme carte de haut niveau et fonctionne pour nos robots sous Ubuntu 16 et ROS Kinetic. Les GPIO sont utilisées pour le contrôle de la vitesse du LIDAR, les afficheurs, la centrale inertielle, ...

[Tout savoir sur le raspberry pi et les GPIO](https://deusyss.developpez.com/tutoriels/RaspberryPi/PythonEtLeGpio/)

## MOSFET et Pont H

Les MOSFETs permettent de commander un élément en puissance à partir d'un signal sortant d'un composant logique. Quand il n'y a pas besoin d'inverser le sens de rotation d'un moteur mais que l'on souhaite le commander en vitessse, c'est notre ami par exemple. Dès que l'on souhaite pouvoir inverser le sens de rotation, on utilise un pont H.

> Le moteur du LIDAR est piloté via un MOSFET
> Les moteurs des roues sont înterfacés via des ponts H

[MOSFET généralité](https://www.electronics-tutorials.ws/transistor/tran_6.html)

[MOSFET utilisation](https://www.electronics-tutorials.ws/transistor/tran_7.html)

[Pont H / L298N](https://www.instructables.com/id/Arduino-Modules-L298N-Dual-H-Bridge-Motor-Controll/)

## Capteur de distance à ultrason / HC-SR04

Les robots sont équipés de LIDAR qui mesurent précisément les distances. Néanmoins, les mesures du laser (LIDAR) sont impossibles sur certains matériaux réfléchissants comme le verre. Aussi, il est bon de compléter le dispositif de détection d'obstacles par des capteurs ultrason. Ils sont moins précis et moins rapides, mais fonctionnent sur les matériaux que le LIDAR ignore.

Il existe de nombreux modules ultrason. Le HC-SR04 propose un excellent rapport qualité / prix. Les modules plus onéreux sont souvent plus précis, avec un angle de détection plus étroit et dispose de fonction d'encodage pour éviter d'être perturbés par d'autres sources d'ultrason.

> Les ultrasons équipent les robots pour de la détection périmètrique d'obstacles

[HC-SR04](https://www.memorandum.ovh/tuto-arduino-utiliser-un-module-ultrason-hc-sr04/)

## Capteur de distance à infrarouge / SHARP

Le capteur infrarouge (IR) est complémentaire de l'ultrason. Il  a l'avantage d'un cône de détection très étroit. Malheuresement, ce type de capteur, pour les version d'entrée de gamme, sont rapidement brouillé ou imprécis en présence de sources de lumière intenses.

Leur utilisateur pour valider la présence/absence d'un objet à distance ou mesurer des distances lors de tâches est bien plus aisé qu'avec des capteurs ultrason. L'inconvénient est leur coût, plus cher que les capteurs à ultrason. Ils sont moins adapté pour la détection d'obstacles car leur cône de mesure étroit nécessite d'utiliser un plus grand nombre de capteurs.

> Les capteurs infrarouge sont utilisé pour détecter la présence d'objets de manière précise et sûre

[Capteur IR de marque SHARP, les choisir et les mettre en oeuvre](https://www.pobot.org/Capteur-IR-Sharp-GP2D120.html)

## Codeur incrémentaux

Mesurer les rotations des moteurs est essentiel pour piloter leur position ou leur vitesse. 

> Le robot utilise des capteurs angulaires connectés aux axes des moteurs pour en mesurer la vitesse de rotation et ainsi pouvoir la contrôler.

[Encodeur incrémentaux / Signaux en quadrature](https://en.wikipedia.org/wiki/Incremental_encoder#Quadrature_outputs)

## Centrale inertielle (ou IMU ou inertial measurement unit)

En mesurant la rotation des moteurs de la base roulante, le déplacement des robots peut être estimé. C'est l'odométrie. Malheureusement, le contact roue/sol n'est ni ponctuel ni permanent (glissements). Aussi, petit à petit, cette estimation de du déplacement réel du robot dérive par rapport à la réalité. Pour améliorer la précision de l'estimation de la pose du robot (la pose c'est la position et l'angle), on s'appuie en plus sur une centrale inertielle qui nous fournit la vitesse angulaire et l'accélération linéaire du robot. On l'appele inertielle car en pratique, on mesure le mouvements de poids miniatures attachés à des ressorts. 

> L'ensemble des mesures, odométrie et en provenant de la centrale inertielle sont fusionnées au travers d'un filtre numérique (UKF)

[Se repérer dans un espace intérieur, problématique et solutions (Avancé)](http://www.captronic.fr/docrestreint.api/328/a8da4127d742fdecabcb4fe62d93027d83db2dc4/pdf/localisation_captronic_vf.pdf)

[Filtre de fusion de capteurs UKF (Avancé)](https://www.seas.harvard.edu/courses/cs281/papers/unscented.pdf)

## Régulateur PID 

Le Graal du roboticien amateur, régler son PID. Mais qu'est ce que ceci signifie ?

> Chaque moteur du robot est asservi en vitesse via un algorithme PID

[Cours de l'équipe RCVA](https://github.com/julienbayle/stardust/raw/master/docs/pdf/rcva-asservissement.pdf)

[Comprendre les PID](http://brettbeauregard.com/blog/category/pid/)

[Comprendre la documentation d'un moteur](https://medium.com/pollenrobotics/how-to-read-a-dc-motors-datasheet-f70fa440452b)

## FreeCAD ou SolidWorks

Certains membres de l'équipe utilise FreeCAD (gratuit et open source) et d'autres SolidWorks pour modéliser le robot durant la phase de conception. L'un est gratuit et pas toujours commode. L'autre est la référence mais il faut demander un licence étudiant.

## Fritzing

Un outil simple pour faire de petites cartes électroniques.

## ROS

ROS est un framework pensé pour la robotique mobile. Une excellente base pour réaliser des codes durables et éviter de réinventer la roue. Il est possible de coder en C++ ou en python. Le C++ reste conseillé pour les programmes gourmands en ressources.

Pour commencer, rien de tel ques les tutoriaux débutant et intermédiaire (si possible en anglais car la version fraçaise est mons qualitive et complète). Mais il existe aussi d'excellentes introductions, morçeaux choisis :

[Introduction](http://homepages.laas.fr/ostasse/Teaching/ROS/rosintro.pdf)

[Débuter avec ROS](http://wiki.ros.org/ROS/Tutorials)

[Présentation du framework](https://fr.slideshare.net/narrendar/ros-an-opensource-robotic-framework)

[Cours progressif en vidéo](https://www.youtube.com/watch?v=0BxVPCInS3M)

[Debugger un programme ROS, les outils](https://bluesat.com.au/a-dummys-guide-to-debugging-ros-systems/)

ROS, c'est ensuite de nombreux modules. Pour comprendre tout le fonctionnement du robot, il faut apprendre le fonctionnement de chaque module utilisé. C'est un périple. Alors, on y va petit à petit ou alors, on se concentre sur ce qui nous est utile au vu de la contribution que l'on souhaite faire.

Au coeur des déplacements du robot, le module de navigation :

[ROS - Navigation, trajectoire et cartes](http://wiki.ros.org/navigation/Tutorials)

Une action est une commande qui dure longtemps, dont on peut suivre l'avancement et qui peut réussir ou échouer :

[ROS - Les actions](http://wiki.ros.org/actionlib_tutorials/Tutorials)

Le LIDAR renvoie des données. Pour transformer les données en estimation de la pose du robot ou deviner la présence du robot adverse, il faut aimer les maths. Quelques modules pour mieux comprendre les enjeux :

[ROS - MRPT](http://wiki.ros.org/mrpt_localization)
[ROS - AMCL](http://wiki.ros.org/amcl)
[ROS - Obstacle detector from LIDAR](https://github.com/tysik/obstacle_detector)

Comment passer de la mesure d'un capteur à la mise à jour de la carte des obstacles ? ROS doit savoir où est le capteur dans le robot. Pour celà, il y a un language de description de robot, l'URDF. Mais un capteur est susceptible de bouger dans le robot (par exemple un capteur monté sur un servomteur). Aussi, pour palier les limites de l'URDF qui est une description statique, il y a les TF (transformés). Elles fournissent un mécanisme pour mettre à jour dynamiquement la position des éléments qui bougent relativement aux autres.

[ROS - URDF](http://wiki.ros.org/fr/urdf/Tutorials/Create%20your%20own%20urdf%20file)
[ROS - TF](http://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf)

Le robot se déplace, c'est pourquoi il y a plein de modules dédiés au contrôle commande des robots. On utilise ROS Control qui est compatible avec GAZEBO que nous utilisons pour les simulations.

[ROS Control](http://www.theoj.org/joss-papers/joss.00456/10.21105.joss.00456.pdf)

Exemples de projet complets avec ROS pour inspiration :

[Linorobot](https://github.com/linorobot/linorobot)




## GAZEBO

Sans simulateur, le développeur a besoin d'un robot et d'une table de jeu pour voir son code prendre vie. Gloire à toi GAZEBO qui nous permet de coder sans robot physique à porté !

> Les deux robots sont simulés sous GAZEBO afin de mettre au point leur fonctionnement

[Gazebo avec ROS Control](http://gazebosim.org/tutorials/?tut=ros_control)

## Behaviour tree

Programmer l'*intelligence du robot* n'est pas une tâche aisée. La meilleure solution connue à l'heure actuelle est le **behaviour tree**. Ce mode de fonctionnement permet la mise en oeuvre d'un programme déterministe et réactif. A découvrir d'urgence !

> Le comportement des deux robots est représenté par un arbre de comportement (behaviour tree)

[Behaviour tree - Savoir les utiliser](https://arxiv.org/pdf/1709.00084.pdf)

[Behaviour tree avec ROS)(https://github.com/miccol/ROS-Behavior-Tree)

## Android pour ROS

Et si l'on connectait un téléphone ANDROID au robot pour bénéficier de sa caméra et son écran ? Un projet de long terme pour ceux qui veulent aller plus loin.

[Cours ANDROID](http://www.univ-orleans.fr/lifo/Members/Jean-Francois.Lalande/enseignement/android/cours-android.pdf)

[ROS - JAVA Android (ouvrir l'onglet kinetic)](wiki.ros.org/rosjava)

[ROS - JAVA Exemples](https://github.com/rosjava/android_apps)