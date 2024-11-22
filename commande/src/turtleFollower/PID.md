# PID numérique

Ce document détail les différents points clés pour réaliser un PID numérique.  

Dans un premier temps, nous rappelons le schéma block d'un PID : 

![Schema block PID](/commande/src/turtleFollower/images/PID.png "Schéma bloc PID")

Dans ce document, nous prenons l'exemple d'un cas d'étude d'un asservissement en position d'un robot mobile. Nous allons donc réaliser un PID pour pouvoir réaliser un tel asservissement.  
## Calcul de l'erreur 

Dans un premier temps, il est nécessaire de calculer l'erreur entre la position actuellement $p$ du robot et la position cible $p_c$.  
Soit $p = p_x \cdot x + p_y \cdot y$ et $p_c = p_{c_x} \cdot x + p_{c_y} \cdot y$, alors nous avons : 
$$
    \epsilon = \sqrt{(p_{c_x}-p_{x})² + (p_{c_y}-p_{y})²}
$$

## Calcul du terme proportionnel 

Le terme proportionnel correspond seulement à la mutliplication du gain proportionnel $K_p$ avec l'erreur actuelle $\epsilon[k]$.
$$
    P = K_p \cdot \epsilon[k]
$$

## Calcul du terme intégral

Pour réaliser une intégration numérique, nous allons multiplier l'erreur actuelle (instant $[k]$) par la période d'échantillonage. De cette façon, plus le temps s'écoulera, plus l'action intégrale sera importante; nous reproduisons bien le comportement voulu. Appelons ce terme $\epsilon_I$ :
$$
    \epsilon_I = \epsilon[k] \cdot T_e
$$

Ainsi, notre terme intégrale sera le suivant : 
$$
    I = K_I \cdot \epsilon_I
$$

## Calcul du terme dérivé

Pour réaliser une dérivation numérique, nous soustrayons l'erreur actuelle (instant $[k]$) avec l'erreur à la période précédente ($ie$ à l'itération précédente $[k-1]$) puis nous dévisons le tout par la période. Appelons ce terme $\epsilon_D$ :
$$
    \epsilon_D = {\epsilon[k] - \epsilon[k-1] \over T_e}
$$

Ainsi, notre terme intégrale sera le suivant : 
$$
    D = K_D \cdot \epsilon_D
$$ 

## Commande de position

D'après le schéma bloc, nous pouvons donc déduire que la commande en position est la suivante : 

$$
    U[k] = P[k] + I[k] + D[k] = K_p \cdot \epsilon[k] + K_I \cdot \epsilon[k] \cdot T_e + K_D \cdot {\epsilon[k] - \epsilon[k-1] \over T_e}
$$