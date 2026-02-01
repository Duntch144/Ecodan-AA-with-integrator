# Contrôle Auto-Adaptatif

Ce document décrit le fonctionnement du mode de contrôle "Auto-Adaptatif" pour la pompe à chaleur Ecodan. Cette fonctionnalité vise à optimiser le confort et l'efficacité énergétique en ajustant dynamiquement la température de l'eau de chauffage en fonction des besoins réels de votre habitation.

L'algorithme se base sur un principe de **contrôle par Delta-T** piloté par un **contrôleur PID**, ce qui permet une régulation plus fine et réactive qu'un simple thermostat ou une loi d'eau basique.

**Note Importante :** Pour que ce mode fonctionne correctement, la pompe à chaleur doit être configurée en mode **"Température de départ fixe"** (`Fixed Flow Temperature`) pour le chauffage.

## Principes de Fonctionnement

### Le Contrôle par Delta-T

Le cœur de la stratégie auto-adaptative est de ne pas imposer une température de départ d'eau fixe, mais de la calculer en temps réel. Pour cela, l'algorithme mesure la température de retour de l'eau du circuit de chauffage et y ajoute une différence de température calculée, appelée "Delta-T".

`Température de Départ Cible = Température de Retour + Delta-T Cible`

Le `Delta-T Cible` n'est pas constant. Il est calculé dynamiquement par le contrôleur PID pour répondre précisément à la demande de chauffage de la maison.

### Le Contrôleur PID (Proportionnel, Intégral, Dérivé)

Le PID est un mécanisme de contrôle intelligent qui ajuste constamment le `Delta-T Cible` en se basant sur l'erreur entre la température ambiante souhaitée (votre consigne) и la température ambiante réelle.

- **Terme Proportionnel (P) :** C'est la réponse directe à l'erreur actuelle. Si votre pièce est à 19.5°C pour une consigne de 20°C, l'erreur de -0.5°C génère un Delta-T de base. C'est la force de réaction principale.

- **Terme Intégral (I) :** Il corrige les erreurs persistantes à long terme. Si, malgré le terme P, la température stagne à 19.8°C sans jamais atteindre les 20°C (erreur de "steady-state"), l'intégrateur va progressivement augmenter le Delta-T pour combler cet écart.
    - **Anti-Windup :** Des sécurités ("anti-windup") empêchent ce terme de croître indéfiniment, notamment lorsque la PAC est déjà à sa puissance maximale ou minimale, évitant ainsi un comportement instable.
    - **Persistance :** La valeur de ce terme est sauvegardée en mémoire non-volatile (NVS). Cela signifie que l'optimiseur n'a pas besoin de "ré-apprendre" le comportement thermique de votre maison après un redémarrage.

- **Terme Dérivé (D) :** Il anticipe le futur en regardant la vitesse de changement de la température. Il peut accélérer la réponse si la température chute rapidement. *Note : Dans l'implémentation actuelle, ce terme est présent mais désactivé par défaut (`DERIVATIVE_GAIN = 0`).*

Le `total_bias` visible dans les logs est la somme des actions de ces termes, et est utilisé pour ajuster la consigne interne avant le calcul final du Delta-T.

### Profils de Chauffe : Courbe en S vs. Linéaire

La façon dont l'erreur de température est convertie en Delta-T dépend du **type de système de chauffage** sélectionné. Cela permet d'adapter la réactivité du système.

- **Profil Doux / Courbe en S (Smoothstep) :** Idéal pour les systèmes à haute inertie comme le **plancher chauffant**. La réponse est très douce pour les petites erreurs et s'intensifie de manière non-linéaire lorsque l'erreur augmente. Cela évite les a-coups et favorise la stabilité.

- **Profil Réactif / Linéaire :** Recommandé pour les systèmes à faible inertie comme les **radiateurs**. La réponse est directement proportionnelle à l'erreur, ce qui permet une montée en température plus rapide.

Le choix du profil se fait via le paramètre `heating_system_type`, en choisissant l'option avec ou sans `*`.

## Fonctionnalités Avancées

### Le "Setpoint Bias"

Le paramètre `auto_adaptive_setpoint_bias` permet d'appliquer manuellement un "biais" à la consigne de température. C'est un outil puissant pour des ajustements proactifs :
- **Anticipation du froid :** Appliquez un biais de +0.5°C quelques heures avant une vague de froid pour pré-chauffer légèrement la maison.
- **Compensation d'apports solaires :** Un jour de grand soleil, appliquez un biais négatif (-1.0°C) pour réduire le chauffage et laisser le soleil travailler.

### Logique "Smart Grid"

Cette fonctionnalité optionnelle permet d'interagir avec des signaux externes (comme un surplus de production solaire) pour une gestion énergétique intelligente.

- **Mode Recommandation (Stockage d'énergie) :**
    - **Condition :** Un interrupteur externe (`sg_energy_available`) indique un surplus d'énergie.
    - **Action :** Le système active un mode "recommandation" (`sg_mode_reco`) qui vise à surchauffer légèrement la maison pour stocker cette énergie gratuite dans la masse thermique du bâtiment. La limite de cette surchauffe est définie par `sg_storage_offset`.

- **Mode OFF (Sécurité Surchauffe) :**
    - **Condition :** La température ambiante dépasse la consigne de plusieurs degrés (défini par `sg_overheat_offset`), que le retour d'eau est déjà chaud et que la demande de chauffage est minimale.
    - **Action :** Le système force l'arrêt du compresseur (`sg_mode_off`) pour éviter une surchauffe inconfortable et un gaspillage d'énergie. Le compresseur est automatiquement ré-autorisé lorsque la température revient à la normale.

## Paramètres de Configuration

Tous les paramètres sont ajustables en temps réel depuis l'interface Home Assistant.

- **`Auto-Adaptive: Control` :** Active ou désactive l'ensemble de la fonctionnalité.
- **`Auto-Adaptive: Heating System Type` :** Sélectionne le profil de chauffe (Linéaire ou Courbe en S) adapté à votre installation (radiateurs, plancher chauffant, hybride).
- **`Auto-Adaptive: Temperature Feedback Source` :** Permet de choisir si la température ambiante de référence est celle lue par la PAC ou une valeur externe (par ex. un autre capteur de température dans Home Assistant).
- **`Auto-Adaptive: Setpoint Bias` :** Permet d'appliquer une correction manuelle à la consigne (voir ci-dessus).
- **`Max. / Min. Heating Flow Temperature` :** Limites de sécurité haute et basse pour la température de départ d'eau. L'algorithme ne dépassera jamais ces valeurs.
- **`Base Min Delta T` / `Max Delta T` :** Définit la plage de travail (en °C) pour le `Delta-T Cible`.
- **`Integral Gain` :** Règle l'agressivité du terme Intégral du PID. Une valeur plus élevée corrigera plus vite les erreurs de long terme, mais peut potentiellement créer de l'instabilité.
- **`SG Overheat Offset` / `SG Storage Offset` :** Règle les seuils en °C pour la logique Smart Grid.
- **`SG Energy Available` / `SG Mode Off` / `SG Mode Reco` :** Interrupteurs pour l'intégration avec la logique Smart Grid.

## Mise en Route

1.  **Configurer la source de température :** Via `temperature_feedback_source`, choisissez si vous utilisez le capteur de la PAC ou un capteur externe plus précis.
2.  **Régler les températures de sécurité :** Ajustez `maximum_heating_flow_temp` et `minimum_heating_flow_temp` à des valeurs raisonnables et sûres pour votre système.
3.  **Choisir le profil de chauffe :** Sélectionnez le `heating_system_type` qui correspond le mieux à votre installation.
4.  **Activer le contrôle :** Passez le switch `auto_adaptive_control_enabled` sur ON.
5.  **Surveiller et ajuster :** Observez le comportement du système. Vous pouvez surveiller les capteurs `Target Delta T` et `Total Bias` pour comprendre les décisions de l'algorithme. Si nécessaire, ajustez les gains ou les biais pour affiner le confort.