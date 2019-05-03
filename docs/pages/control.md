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


## Asservissement robot principal

### Un souci de poids

L'utilisation du script *calibrate.py* du package *sd_calibrate* revèle que le robot est trop lourd relativement à la puissance de ses moteurs. En effet, le PWM minimal pour faire bouger le robot est compris entre 50 et 60% de sa puissance.

Sur la balance, voici le poids vu pour chaque roue :
 - 2.2 kg à droite
 - 1.9 kg à gauche
 - 2,6 kg à l'arrière

En pratique, les batteries au plomb du robot ont été remplacées par des batteries LIPO et le gain de poids a résolu ce souci.


### Réglage PID

Démarrer le robot et vérifier que la commande à distance par télécommande fonctionne.
Passer le robot en mode "alimentation par batterie" (L'alimentation secteur n'étant pas équivalente, les réglages pourraient être erronés)

Installation de plot juggler:

```bash
sudo apt-get install ros-melodic-plotjuggler
```

Sur un poste distant avec Ubuntu :
export ROS_MASTER_URI=http://192.168.0.27:11311
rosrun rqt_reconfigure rqt_reconfigure &
rosrun plotjuggler PlotJuggler &

Dans PlotJuggler :
Ouvrir : sd_calibration/config/plotjuggler-pid-tuning.xml
Ceci permet de voir les valeurs utiles du contrôle commande des moteurs

Dans rqt_reconfigure :
On met toutes les valeurs à 0 (pour désactiver tous les contrôleurs initialement)


Commençer le réglage :
 
1. Placer le gain "Friction statique" à 1 puis le diminuer jusqu'à ce que la roue ne bouge presque plus suite à une sollicitation du robot à la manette. Attention, en fin de réglage la roue doit toujours pouvoir tourner dans les deux sens (même très lentement). Noter cette valeur ou la reporter immédiatement dans le fichier de configuration du hardware interface du robot, en effet, les gains via rqt_reconfigure sont perdus une fois le robot redémarré.

2. Augmenter le gain "feedfordward" jusqu'à ce que la consigne de vitesse soit grossièrement respectée (à vérifier sur le moniteur graphique plotjuggler). Noter ou enregister le gain obtenu.

3. Affiner le réglage en sélectionnant des valeurs pour le PID de la roue (à vérifier sur le moniteur graphique plotjuggler). Noter ou enregister les gains obtenus.

4. Dans un terminal connecté sur le robot, lancer l'exécutable pour réaliser des trajectoires de validation de l'asservissement :
rosrun sd_calibration move.py et affiner les réglages pour avoir un suivi de consigne qualitatif

Il est possible de faire le réglage roue par roue ou toutes les roues à la fois.
