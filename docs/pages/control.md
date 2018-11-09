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