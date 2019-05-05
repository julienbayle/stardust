# Contrôle commande des robots

La modelisation du robot omnidirectionnel à trois roues est inspirée par l'article de Mariane Dourado Correia [Modeling of a Three Wheeled Omnidirectional Robot Including Friction Models](https://github.com/julienbayle/stardust/raw/master/docs/pdf/omnidirectionnal_model.pdf)

L'article n'est pas exempt de coquilles dans les formules. Néanmoins, il propose un modèle à état pour les robots omnidirectionnel à trois roues intéressant.

Le code développé se base sur les conventions suivantes :

<img src="https://github.com/julienbayle/stardust/raw/master/docs/images/omnibase.png" width="100%" />

Les équations suivantes donnent les principales transformations de vitesse implémentée dans le controller *ThreeOmniWheelsDriveController* du paquet *sd_control*.

Vitesse angulaire des roues <-> Vitesse linéaire au centre des roues :

\\(
\begin{bmatrix}
\dot{\theta_{left}} \\\\ 
\dot{\theta_{right}} \\\\ 
\dot{\theta_{back}}
\end{bmatrix}
= 
\begin{bmatrix}
\frac{1}{r} & 0 &0 \\\\ 
0 & \frac{1}{r} & 0 \\\\ 
0 & 0 & \frac{1}{r}
\end{bmatrix}
\begin{bmatrix}
v_{left} \\\\ 
v_{right} \\\\ 
v_{back} 
\end{bmatrix}
\\)

Vitesse linéaire au centre des roues <-> Vitesse linéraire au centre du robot :

\\(
\begin{bmatrix}
v_{left} \\\\ 
v_{right} \\\\ 
v_{back} 
\end{bmatrix}
=
\begin{bmatrix}
\frac{-\sqrt{3}}{2} & \frac{1}{2} & L \\\\ 
\frac{\sqrt{3}}{2} & \frac{1}{2} & L \\\\ 
0 & -1 & L
\end{bmatrix}
\begin{bmatrix}
v_{x} \\\\ 
v_{y} \\\\ 
\dot{\theta}
\end{bmatrix} 
\\)

Vitesse linéraire au centre du robot <-> Vitesse linéaire au centre des roues :

\\(
\begin{bmatrix}
v_{x} \\\\ 
v_{y} \\\\ 
\dot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
\frac{-\sqrt{3}}{3} & \frac{\sqrt{3}}{3}  & L \\\\ 
\frac{1}{3} & \frac{1}{3} & \frac{-2}{3} \\\\ 
\frac{1}{3L} & \frac{1}{3L} & \frac{-2}{3L} 
\end{bmatrix}
\begin{bmatrix}
v_{left} \\\\ 
v_{right} \\\\ 
v_{back} 
\end{bmatrix}
\\)


Le contrôle commande est inspirée de l'article de Humberto X. Araújo [Model Predictive Control based on LMIs Applied to an Omni-Directional Mobile Robot](https://github.com/julienbayle/stardust/raw/master/docs/pdf/omnidirectionnal_control.pdf)


\\(
\begin{bmatrix}
v_{x} \\\\ 
v_{y} \\\\ 
\dot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
\frac{-\sqrt{3}}{3} & \frac{\sqrt{3}}{3}  & L \\\\ 
\frac{1}{3} & \frac{1}{3} & \frac{-2}{3} \\\\ 
\frac{1}{3L} & \frac{1}{3L} & \frac{-2}{3L} 
\end{bmatrix}
\begin{bmatrix}
v_{left} \\\\ 
v_{right} \\\\ 
v_{back} 
\end{bmatrix}
\\)


## Asservissement

### Etape 1 : Evaluation de la friction statique

Un programme a été réalisé permettant de déterminer le PWM minimum pour faire tourner une roue. Il s'agit de *eval_static_friction*. 

Exemple d'appel pour la roue droite du robot 2 avec une recherche du PWM minimal dans la plage PWM = 30 ... 70 :

```bash
rosrun sd_calibration eval_static_friction.py /r2/right/encoder/speed /r2/right/pwm 30 70
```

La valeur obtenue doit être divisé par le PWM max et appliquée dans le réglage de la friction statique du paquet sd_harware_interface. Ici pour la roue droite du robot 2, le fichier r2_hardware_interface.yam / velocity_controllers_static_friction_right

### Etape 2 : Réglage des vitesses et accélérations limites

Le paquet *sd_control* a pour rôle de transformer une commande en vitesse au niveau du robot, en commandes en vitesse pour les roues (vitesses angulaires), en respectant les vitesses et accélérations limites. 

Ouvrir le fichier rX_control.yaml de ce paquet puis :
 - régler les valeurs physiques du robot
 - fixer les vitesses et accélérations maxi

Ce réglage pourra être affiné ensuite si l'on découvre lors de l'étape "Réglage PID" que le robot ne peut pas atteindre les consignes.

Puis ouvrir le fichier rX_teleop.yaml du paquet *sd_teleop* et reporter les valeurs choisies.

### Etape 3 : Réglage des encodeurs

Le paquet *sd_harware_interface* a pour mission d'appliquer les vitesses angulaires cibles au niveau des roues. La première étape est d'étalonner les encodeurs et régler les topics.

Pour valider que le réglage est bon, on pousse le robot à la main et on valide que l'odométrie est correctement calculée (position, orientation et vitesse).

```bash
rostopic echo /r2/mobile_base_controll/odom
```

### Etape 3 : Réglage PID

Démarrer le robot et vérifier que la commande à distance par télécommande fonctionne.
Passer le robot en mode "alimentation par batterie" (L'alimentation secteur n'étant pas équivalente, les réglages pourraient être erronés)

Installation de plot juggler:

```bash
sudo apt-get install ros-melodic-plotjuggler
```

Sur un poste distant avec Ubuntu (dans le dossier scripts du projet):

```bash
source ./source-pc-slave.ph <nom_du_robot>
rosrun rqt_reconfigure rqt_reconfigure &
rosrun plotjuggler PlotJuggler &
```

Dans PlotJuggler, charger le layout *sd_calibration/config/plotjuggler_pid_tuning_rX.xml* puis passer le buffer size à 20 secondes.

Ainsi configuré, PlotJuggler va permettre de voir les valeurs utiles du contrôle commande des moteurs.

Dans rqt_reconfigure :
On met toutes les valeurs à 0 (pour désactiver tous les contrôleurs initialement) sauf la friction statique calculée précédement.

Commençer le réglage :
 
1. Augmenter le gain "feedfordward" jusqu'à ce que la consigne de vitesse soit grossièrement respectée (à vérifier sur le moniteur graphique plotjuggler). Noter cette valeur ou la reporter immédiatement dans le fichier de configuration du hardware interface du robot, en effet, les gains via rqt_reconfigure sont perdus une fois le robot redémarré.

2. Affiner le réglage en sélectionnant des valeurs pour le PID de la roue (à vérifier sur le moniteur graphique plotjuggler). Noter ou enregister les gains obtenus.

3. Dans un terminal connecté sur le robot, lancer l'exécutable pour réaliser des trajectoires de validation de l'asservissement (le premier paramètre est le nom du robot, le second, la valeur limite de la vitesse linéaire en m/s et dernier, la vitesse angulaire en rad/s):

```bash
rosrun sd_calibration pid_tuning_move.py r2 0.2 3
```

Affiner les réglages pour avoir un suivi de consigne qualitatif. Il est possible de faire le réglage roue par roue ou toutes les roues à la fois.

Exemple de réglage :

<img src="https://github.com/julienbayle/stardust/raw/master/docs/images/pid_tuning.png" width="100%" />