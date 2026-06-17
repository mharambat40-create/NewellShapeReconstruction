# Direction artistique de l'application

## Synthese

L'application adopte une direction artistique sobre, technique et pedagogique. Le langage visuel est celui d'un outil de visualisation 3D plutot que d'une interface "marque" ou decorative.

Les choix dominants sont :

- une interface Qt native, sans feuille de style custom, donc avec l'apparence standard du systeme d'exploitation ;
- une grande zone OpenGL a gauche pour mettre la geometrie au premier plan ;
- un panneau de controle vertical a droite, structure en blocs fonctionnels ;
- une palette neutre gris clair pour la scene ;
- quelques accents tres lisibles pour les reperes spatiaux, le survol et la lumiere.

En pratique, l'identite visuelle repose surtout sur le rendu 3D et non sur l'habillage des widgets.

## Composition de l'interface

La composition generale est la suivante :

- a gauche : la scene 3D OpenGL ;
- a droite : un panneau de controle scrollable, divise en trois groupes ;
- groupes fonctionnels : `Objets et animation`, `Transformation`, `Rendu` ;
- structure principale : `QSplitter` horizontal avec une scene dominante et un panneau secondaire plus etroit.

Cette organisation produit une interface d'outil de travail, claire et rationnelle, avec une separation nette entre visualisation et parametres.

## Palette visible dans l'application

Les couleurs ci-dessous sont celles effectivement visibles dans le rendu actuel.

| Couleur | Hex approx. | Usage | Application |
| --- | --- | --- | --- |
| Gris tres clair | `#F8F8F8` | Fond principal | Arriere-plan de la scene OpenGL |
| Gris bleute clair | `#D6DBE0` | Matiere principale | Faces des solides et modeles affiches |
| Cyan tres pale | `#E7FFFF` | Accent d'interaction | Face survolee a la souris |
| Gris anthracite | `#3C4146` | Structure / lisibilite | Contours visibles, wireframe, lignes de lecture |
| Violet vif | `#934BEF` | Accent spatial secondaire | Axe de rotation de l'objet |
| Rouge | `#D91F1F` | Reperage spatial | Axe monde X |
| Vert | `#1FAD2E` | Reperage spatial | Axe monde Y |
| Bleu | `#245CE6` | Reperage spatial | Axe monde Z |
| Jaune creme | `#FFF5B8` | Indicateur de lumiere | Sphere representant la source lumineuse |

## Lecture artistique de la palette

La palette suit une logique tres fonctionnelle :

- les tons neutres servent a ne pas concurrencer la geometrie ;
- les accents colores servent uniquement a informer ;
- la couleur n'est pas utilisee pour "decorer" l'interface, mais pour orienter la lecture spatiale ;
- le contraste principal oppose surfaces claires, contours fonces et reperes colores ;
- le survol emploie un cyan tres pale, donc visible sans casser l'equilibre general.

Cela donne une ambiance de laboratoire, de viewer technique ou de prototype d'enseignement en visualisation.

## Elements et couleurs associes

### 1. Scene et objet principal

- Fond de scene : `#F8F8F8`
- Faces du solide courant : `#D6DBE0`
- Contours / wireframe : `#3C4146`
- Face survolee : `#E7FFFF`

Effet visuel :

- rendu clair, lisible, peu agressif ;
- la forme est d'abord comprise par la lumiere et les contours ;
- l'accent de survol reste doux, presque clinique.

### 2. Reperes spatiaux

- Axe objet : `#934BEF`
- Axe monde X : `#D91F1F`
- Axe monde Y : `#1FAD2E`
- Axe monde Z : `#245CE6`

Effet visuel :

- codage spatial immediat et standard ;
- l'axe objet est volontairement distinct des axes monde grace au violet ;
- les reperes sont plus expressifs que l'objet lui-meme.

### 3. Source lumineuse

- Sphere de lumiere : `#FFF5B8`

Effet visuel :

- la source lumineuse est percue comme chaude et legere ;
- elle se distingue du gris froid de la scene sans devenir dominante.

### 4. Interface Qt

Les widgets Qt ne recoivent pas de palette custom dans le code actuel. Cela signifie :

- menus, boutons, cases a cocher, labels, sliders et champs numeriques utilisent le theme natif du systeme ;
- l'identite visuelle du panneau lateral depend surtout de la structure, des espacements et des groupements, pas d'une charte couleur maison.

## Couleurs presentes dans le code mais non dominantes a l'ecran

Certaines couleurs sont stockees dans les maillages importes ou generes, mais ne pilotent pas le rendu principal actuel, car le maillage actif est dessine avec une couleur uniforme (`kSolidColor` pour les faces, `kVisibleEdgesColor` pour les contours).

| Couleur | Hex approx. | Origine | Statut actuel |
| --- | --- | --- | --- |
| Bleu clair | `#A6C7F2` | Couleur attribuee aux sommets des OBJ et STL importes | Presente dans les donnees, mais non visible dans le rendu principal actuel |
| Jaune soutenu | `#FFEB40` | Couleur de `makeArrow()` | Definie dans le code, mais la fleche n'est pas utilisee dans la scene actuelle |
| Jaune tres clair | `#FFFAB8` | Couleur interne du maillage sphere | Definie dans le maillage, mais la sphere visible est finalement rendue avec `#FFF5B8` |

## Effets de rendu qui participent a la direction artistique

Au-dela des couleurs, le rendu contribue fortement a l'esthetique :

- eclairage diffus + speculaire leger ;
- ambient doux pour conserver les volumes lisibles ;
- overlay de contours pour clarifier la lecture des formes ;
- antialiasing configurable ;
- camera orbitale centree sur l'objet.

Le resultat vise la comprehension volumique et la precision de lecture plutot que le photorealisme.

## Sources code

- Structure de l'interface : [mainwindow.cpp](/Users/martin/Desktop/THE_CUBE%20test%20mac/mainwindow.cpp)
- Palette principale et rendu OpenGL : [glwidget.cpp](/Users/martin/Desktop/THE_CUBE%20test%20mac/glwidget.cpp)
- Couleurs stockees dans les maillages : [meshfactory.cpp](/Users/martin/Desktop/THE_CUBE%20test%20mac/meshfactory.cpp)
