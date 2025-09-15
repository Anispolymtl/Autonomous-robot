INF3995 – Projet de conception d’un système informatique
Rapport d'avancement de projet – Équipe 106 

Remise : 15 septembre 2025


Tâches réalisées durant la semaine:
Anis Menouar
1. Tutoriel ROS2.
2. Mise en place de l’environnement de simulation Gazebo et de programmation ROS2.
3. Première utilisation avec les robots physiques dans les laboratoires.
4. Analyse des codes « Templates » mise à notre disposition sur Github par les chargés.
5. Avancement de l’analyse et de la complétion de certaines questions du PDR.
6. Analyse et meilleure compréhension de l’architecture générale et logicielle du robot (Front end, middlewear ROS, backend et robot physique)

Arnaud Grandisson 
1. Tutoriel ROS2
2. Installation des logiciels nécessaires au cours
3. Prise en main du robot
4. Prise en main de la simulation
5. Mise en forme du git et des fichiers pour démarrer
6. Analyse des fichiers des lauchs files exemples et de limo.

Matis Roux
1. Familiarisation avec outils pour utiliser ROS2
2. Début du développement du UI du site web
3. Apprentissage du fonctionnement de la connexion au robot
4. Compréhension des principes de base du fonctionnement de ROS2

Sarah Aksas
1. Installation et configuration des outils (Ubuntu 22.04, ROS2, Docker, Gazebo)
2. Tests initiaux avec le robot physique en laboratoire.
3. Création et structuration détaillé du plan d’action de l’équipe.
4. Rédaction d’un document de planification et brainstorming collectif.
5. Organisation du GitLab de l’équipe :
création et structuration des issues selon les consignes officielles (objectifs, critères de complétion, preuves attendues),
mise en place et clarification de l’usage du tableau Kanban GitLab (colonnes, suivi hebdomadaire, cohérence avec les rapports),
réflexion des milestones et des règles d’association (assigné·e, poids, échéance),
création des dossiers requis à la racine (weekly-reports/, demos/).
6. Contribution à la préparation des contenus pour la Documentation du projet (PDR).

Félix Paillé Dowell
Tutoriel ROS2
Tests de mouvement et de connexion aux robots.
Complétion de la question 2 du PDR 
Planification pour un premier script sur le robot
Planification de l’intégration du nouveau script avec le site web croquis.
Patrick Léonnel Nzudom Ketchateu
Tutoriel ROS2.
Première utilisation avec les robots physiques dans les laboratoires.
Complétion de deux questions pour le PDR.


Les enjeux majeurs rencontrés :
La faible connaissance de l’environnement nous a fait prendre du temps pour commencer pour suivre des tutoriels.
C’est assez nouveau pour nous de mener un projet de grande envergure du début à la fin c’est une adaptation supplémentaire en terme d’organisation et de distribution du temps



Le plan d’action pour la semaine en cours :

1. Alignement technique
Vérifier que toute l’équipe partage la même compréhension de l’architecture physique (robots AgileX Limo, station au sol, simulation Gazebo) et de l’architecture logicielle (serveur web, conteneurs Docker, ROS2).
Discuter des standards de programmation et d’intégration (style de code, conventions, GitLab).
2. Manipulation et tests préliminaires
Prendre en main le robot physique et valider sa connexion au serveur web via le réseau fourni.
Explorer les fonctionnalités de base exigées (ex. commande Identifier – R.F.1).
3. Premiers fichiers Launch (ROS2)
Continuer à créer et tester des fichiers .launch.py permettant de lancer différents packages.
Utiliser ces fichiers comme base pour atteindre rapidement les requis R.F.1 et R.F.2 (démonstrations PDR).
4. Simulation et outils
Continuer à suivre des tutoriels ROS2 et analyser les scripts existants (.py) sur GitLab pour s’approprier le projet.
Assurer la compréhension de la simulation Gazebo (lancer un environnement, manipuler deux robots).
5. Docker & intégration
Explorer l’usage de Docker Compose pour organiser les services (un launch file par service, ou un global).
Définir une structure de travail commune avec Docker pour respecter le requis de conteneurisation obligatoire.
6. Identification et interaction
Étudier comment implémenter l’identification d’un robot (clignoter une DEL, son, mouvement) pour satisfaire R.F.1.
Définir les scénarios de connexion serveur ↔ robot physique (1) et serveur ↔ simulation (2).
7. Documentation et planification
Poursuivre la rédaction d’une première version de la Documentation du projet (réponse à l’appel d’offres).
Préparer l’organigramme technique, le plan de projet et l’échéancier graphique, conformément à l’Annexe B de l’appel d’offres.


