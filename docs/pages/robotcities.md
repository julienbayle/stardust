# Retour d’expérience 2018 (Eurobot / Robot cities)

L'équipe s'est formée en décembre 2017. C'était donc notre première participation à la [coupe de robotique Eurobot](https://www.coupederobotique.fr).

Nous avons fini 75ème. Nous sommes très fier de notre résultat et remercions notre sponsor, l'[agence de communication Kelcom, basée à Nantes](http://www.kelcom.fr), pour les élégants [polos marqués](https://www.objets-publicitaires-pro.com/) offerts.

Cette année-là, 133 équipes avaient réussi à homologuer leurs robots sur près de 200 équipes inscrites. 

<img src="https://github.com/julienbayle/stardust/raw/master/docs/images/match2018.jpg" width="400" />

Les robots étaient presque prêts à notre arrivée à la coupe. L'abeille et le panneau domotique étaient prêts. Autrement dit, nous n'étions pas tout à fait prêts pour la phase d'homologation. Sur le robot principal, chaque chose fonctionnait indépendamment. Il restait "juste" à tout faire fonctionner ensemble. Sur le robot secondaire, il fonctionnait à la télécommande mais pas encore en mode autonome.

Quelques derniers efforts plus tard, nous étions homologués, toute juste après la première série de matchs. Nous avons joué 4 matchs. Mais attention... 4 matchs et 4 victoires !

Ce document est un petit retour d'expérience pour partager avec les prochains membres de l'équipe ainsi que toute la communauté de roboticiens amateurs.

[Règles du concours 2018 robot cities](https://github.com/julienbayle/stardust/raw/master/docs/pdf/rules2018.pdf)

## Se fixer des limites

Qui dit robot pas prêt, dit robot à terminer, dit équipe centrée sur elle. 

Pour profiter pleinement de l'évènement, il préférable de ne faire que des ajustements sur place afin de pourvoir profiter de l'évènement, suivre les matchs des autres équipes, profiter des conférences et bien sûr passer du bon temps avec les autres. 

Pour avoir un robot fonctionnel à la coupe, il faut savoir se fixer des limites raisonnables associées à un macro planning de réaliste. Pour éviter qu'une étape ne pousse les prochaines, le planning doit être régulièrement et collectivement arbitré par l'équipe. 

L'objectif ne se résume pas en **faire le plus de points possible**. Il est important de **prendre plaisir ensemble et réussir à ce que chacun trouve sa place** et **arriver sur place pour présenter sa réalisation plutôt que la terminer à force de travail et de courtes nuits**.

## Avoir un simulateur

La mise au point des trajectoires et de la partie comportementale des robots sans simulateur est pénible. Chaque modification du code, pour être validée avant un match, nécessite un test sur une table préparée en condition de match. Et deux fois, car il faut tester que cela fonctionne des deux côtés. Chaque validation prend un temps conséquent et nécessite d'avoir robots et table à disposition.

Une fois sur place, c'est encore pire car l’accès aux tables de tests officielles est synonyme de faire la queue en moyenne 20 minutes. Puis un seul test... qui peut être un flop. 

De plus, un simulateur permet de simuler les robots adverses et les deux robots en simultané afin de vérifier qu'il n'y a pas de blocage mutuel entre les robots, ce que les tests sur table ne permettent pas.

## Avoir sa propre table

Ce fût un vrai plus pour nous. Peu d’équipes en disposait en pratique et nous n'aurions pas eu le temps de finaliser les codes du robot sans. C'est aussi un élément de décor qui crée l'ambiance "coupe" à la maison lors de la mise au point des robots et de autres éléments du jeu.

## Travailler régulièrement et commencer tôt

Participer à la coupe Eurobot est un travail d'équipe. Pour se classer dans le haut du tableau, il faut compter entre 1000 et 5000 heures de travail de préparation.

A titre l'illustration 1000 heures, 6 personnes, 8 mois, c'est une moyenne de 5 heures par semaine et par personne. 

Entre la logistique, la conception, la construction de tous les dispositifs, la programmation, la mise au point, l'acquisition de connaissances, la documentation et les échanges en équipe, 1000 heures c'est peu de choses.

En 2018, l'investissement de toute l'équipe avoisine les 3000 à 4000 heures. Et nous n'avons pas réussi à conclure. L'ambition était là, si tout avait fonctionné nous aurions été en haut du tableau.

L'idéal est de se fixer un rythme et s'y tenir au long de l'année. L'essentiel est de le tenir dans la durée. Ce n'est pas un sprint mais un marathon. De plus, tout travail mené au dernier moment est bien souvent du temporaire qui ne constitue pas un socle valable pour les années à suivre.

## Fixer des exigences

Avoir une liste des exigences pour les robots permet de partage en équipe les objectifs et les contraintes et enfin de vérifier régulièrement si l'on n'est pas en train de dériver.

## Pendant la coupe : les limites du WIFI

La programmation du robot en WIFI pendant l'évènement est difficile, il est préférable de pouvoir passer en mode filaire au besoin. La communication entre l'ordinateur et le raspberry pi n'était possible qu'en mettant un téléphone dans le robot pour faire relais.

L'ajout d'une antenne WIFI au raspberry pi est également une possibilité.

## Composants bon marché

L'utilisation de composants bon marché signifie économie. Oui !

Néanmoins, gare à l'excès avec des composants fragiles ou manquant de fiabilité. Ainsi, un servomoteur est devenu fou à cause des ondes électromagnétiques sur place (il bougeait tout seul avec une commande stable). Remplacé, nous en avons grillé deux autres car à chaque démarrage du robot, le zéro du servomoteur se décaler. Heureusement qu'une autre équipe a pu nous dépanner.

C'est toujours pénible de voir le travail de toute une équipe ne pas pouvoir donner de résultats à cause d'un élément défectueux impossible à réparer ou remplacer sur place.

A utiliser avec modération et prévoir du stock de remplacement.

## Robotique amateur

C'est une coupe de robotique **amateur**. Oui, il y a du travail. Oui, il faut faire des choses propres et robustes. Mais il y a la place pour du "fun".

A ce titre, la prise en compte du côté scénique et du design des robots est très importante. Sur place, emmener des bonbons ou décorer le stand est essentiel. Avoir des éléments décalés sur les robots... c'est permis.

L'an dernier, entre la phase **robot en bois** et **robot peint**, ce n'était plus du tout le même univers. Nous aurions dû prendre beaucoup plus tôt ce temps pour personnaliser les robots. Secondaire ou principal, ce qui est sûr, c'est qu'il ne faut pas l'oublier.

## Bonne pratique observée chez les autres : Faire son zéro

Le robot fait son zéro tout seul en début de match. On le pose sur la table, on met la tirette et il revient à sa place. C'est pro !

## Bonne pratique observée chez les autres : Détection arrêt d'urgence enclenché

Quand on appui sur l'arrêt d'urgence, les moteurs se coupent mais pas le programme.
Idéalement, il serait bon que le robot arrête son programme en cas d'appui sur l'arrêt d'urgence.
Ainsi, chaque appui revient à faire faire un reset du programme du robot et pas besoin de l'éteindre et le rallumer pour remettre le programme à zéro.

## Bonne pratique observée chez les autres : Utilisation d'un bus de données

Certaines équipes disposent de blocs réutilisables d'année en année basées sur un bus standardisé (communication et alimentation en 3,3V, 5V et 12V). Le plus souvent, le protocole est un standard des bus terrain industriels (RS485, ModBus, CAN, I2C, SPI). Ces équipes profitent également de modules industriels compatibles avec le protocole retenu.

Exemple de blocs :
- 1 moteur avec boucle de contrôle PID en vitesse ou position et retour des informations de position, vitesse, effort (capteur de courant)
- un servomoteur
- module énergie (régulation multi tension, capteur de courant et de tension, alimentation batterie ou externe)
- capteurs de distance ou de contact
- module afficheur

Il est possible de faire des petits modules intelligents soit même en employant des PICs ou des arduinos mini pro avec une puce d'interface selon le protocole choisi (Max485 pour RS485 par exemple).

L'avantage d'un bus est qu'il est possible de relier presque tous les éléments en série, ce qui simplifie le câblage dans le robot.

Le choix du protocole est difficile. L'essentiel, c'est ensuite de s'y tenir.

## Bonne pratique observée chez les autres : Des éléments non remis en cause

Beaucoup d'équipes utilisent la même base chaque année. Elles acceptent celle-ci comme une contrainte lors de leur conception. Si besoin d'une nouvelle base plus performance, elle lance un projet de nouvelle base mais pour l'année suivante. Cette base, souvent appelée "base roulante" intègre le plus souvent les moteurs et la détection d'obstacles (à distance ou en butée).

Certaines équipes comme CoffeMachine, classée cette année 16ème, ne change même presque rien d'année en année.

<iframe width="560" height="315" src="https://www.youtube.com/embed/BT-OIcQiZDQ" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

L'idée est que modifier la base mobile signifie revoir les algorithmes de déplacement, de planification de trajectoire, d'évitement d'obstacle, les réglages des contrôleurs, refaire de la mécanique et de l'électronique, des tests, ... et donc prendre le risque de ne pas réussir à terminer à temps.

## Bonne pratique observée chez les autres : Roue codeuse indépendante

Les roues d'un robot doivent bien adhérer au sol et donc être large et souples. En conséquence, le point de contact roue/sol n'est pas ponctuel, ce qui rend l'odométrie moins précise. Aussi, les équipes matures disposent d'une base roulante avec des roues larges et présentant un bon contact avec le sol (gomme souple), associées à des roues codeuses spécialement pour l'odométrie.

<img src="https://github.com/julienbayle/stardust/raw/master/docs/images/odometry_wheel.jpg" width="400" />

Les roues pour l'odométrie sont très fines et montées sur des ressorts pour garantir le meilleur contact possible avec le sol. L'axe des roues motrices et des roues odométrie est le même afin d'améliorer la précision de l'odométrie lors des rotations.

## Bonne pratique observée chez les autres : Plusieurs cartes mémoires

Des équipes utilisaient deux cartes mémoires. Une première dispose d'un code stable pour aller faire un match. Une seconde permet de mettre au point des améliorations. Dès que les améliorations sont validées, la carte amélioration devient la carte de match et vice et versa.

Ainsi, à tout moment et sans risque, l'équipe peut améliorer son code ou partir pour un match en insérant l'autre carte. Cette année, nous avons souvent été appelé pour un match alors que nous étions en train de modifier le code et donc que nous n'avions pas encore tester les modifications. Surprise lors du match !

## Bonne pratique observée chez les autres : Indicateur de batterie

Beaucoup d'équipe ont étalonné leur batterie de manière à bien connaitre l'autonomie restante.

## Bonne pratique observée chez les autres : Périmètre du robot inférieur à la limite

En construisant un robot avec un périmètre inférieur à la limite du règlement, il devient possible de bidouiller autour lors de la coupe sans avoir à en couper un bout par ailleurs.

Ainsi, beaucoup d'équipes ont pu adopter une solution commune pour allumer le panneau domotique : visser une barre de bois très large à l'arrière du robot pour allumer l'interrupteur via une marche arrière. C'était une solution très simple mais peu d'équipes l'avait trouvé avant d'être sur place. Tous ceux qui avait la place l'ont adopté !

## Bonne pratique observée chez les autres : Un site d'archive par année avec un bilan archivé

Chaque année l'équipe publie ses pensées, codes, schémas... et c'est une ressource pour les suivants. Plus l'équipe est ancienne, plus il est facile de faire un robot en s'appuyant sur l'expertise et l'expérience des membres senior.

Exemple de sites :
- https://www.rcva.fr
- http://xenomorphales.org/
- https://github.com/utcoupe/coupe18

## Et pour finir dans les 16 premiers ?

Faire entre 100 et 200 points par match avec un seul robot ou deux robots simples était du domaine du possible.

**Cas 1 : Uniquement les actions basiques**

Réalisation :
- abeille : 55 pts
- panneau : 30 pts
- 2 cubes joker en pile : 3 pts (intégré dans le robot au démarrage)
- ouverture d'un récupérateur d'eau : 10 pts
- non forfait : 10 pts
- bonus estimation du score : 37 pts

Total : 145 points

Réussite sur 5 matchs : 725 points

Classement final : 9ème

Et oui, réussir uniquement les actions les plus simples, à chaque match, aurait permis de finir dans les 10 premiers. Et un seul robot assez rudimentaire suffisait pour cela.

**Cas 2 : Actions basiques et quelques actions de jeu**

Exemple de réalisation pour faire ce score :
- actions basiques : 145 points
- 6 cubes de plus (au sol) : 6 pts
- ouverture d'un second récupérateur : 10 pts
- 3 balles dans la station d'épuration : 30 pts
- bonus supplémentaire bonne estimation du score : 6 pts

Total : 197 points

Réussite sur 5 matchs : 985 points

Classement final : 4ème

**Conclusion : Rester raisonnable et avoir un système robuste et fiable**

Nous avons marqué 193 points sur 4 matchs. A chaque match, nous avons réussi qu'une seule action (abeille ou panneau).

Comme l'illustre cette étude, il n'est pas nécessaire de faire toutes les actions de jeu pour être qualifié pour les phases finales (16 premiers). 

Sans faire de piles de cubes et sans envoyer les balles dans le château d'eau par balistique, c'était possible.

En pratique, beaucoup de robots s'arrêtent à cause d'un système d'évitement d'obstacle qui met l'éxécution en pause en attendant que la voie soit libre... sauf que le robot adverse fait pareil. 
