# Architecture logicielle générale
<img width="877" height="345" alt="Screenshot 2026-01-13 at 3 04 32 PM" src="https://github.com/user-attachments/assets/d6682a25-9316-44d3-b974-a443b9abde19" />


# Aperçus de l'utilisation des robots & de l'application Web

1.Page de connexion.
<img width="628" height="366" alt="Screenshot 2026-01-13 at 3 05 48 PM" src="https://github.com/user-attachments/assets/43794b3c-ff67-42f0-8f87-1042869fa8e5" />

2.Page principale.
<img width="1081" height="586" alt="Screenshot 2026-01-13 at 3 05 29 PM" src="https://github.com/user-attachments/assets/de81a90b-dc2b-4a6d-bcd5-f0d3c0a85cbb" />

3.Page de journalisation
<img width="646" height="264" alt="Screenshot 2026-01-13 at 3 06 10 PM" src="https://github.com/user-attachments/assets/f2393294-fb1b-43b3-beaf-98d692941be7" />


# 🚀 Guide Général — Démarrage Robot & Application Web

Ce guide regroupe **toutes les étapes** nécessaires pour démarrer les robots et l’application web, de manière **simple et uniforme**.

- **Simulation** → Étape 3  
- **Robots réels** → Étape 1 + Étape 2  
- Puis → Étape 4 (serveur + client)
---

## Étape 1 : Connexion au robot (mode physique)
Exécutez ces étapes **dans chaque terminal** avant de lancer un robot physique :
```bash
ssh -X [votre_user]@lmXXX.local
# Mot de passe (quand demandé) : h2025!

sudo chmod 666 /dev/ttyTHS1
```
> Remplacez `lmXXX.local` par l’identifiant de votre robot (ex. lm1335.local, lm1166.local, etc.).  
> L’option `-X` permet l’export d’affichage X11 si nécessaire.

---

## Étape 2 : Démarrage des robots réels (deux terminaux séparés)

### Terminal équipe201
Dans le dossier `project_ws` :
```bash
colcon build
source install/setup.sh
ros2 launch robot_exploration robot_bringup.launch.py use_limo:=true
```

### Terminal équipe206
Dans le dossier `project_ws` :
```bash
colcon build
source install/setup.sh
ros2 launch robot_exploration robot_bringup.launch.py use_limo:=true namespace:="limo2"
```

---
## Étape 3 : La simulation

Ceci est pour rouler en **mode simulation**.  
⚠️ **Ne pas utiliser pour le mode physique**. Voir **Étape 1 et 2** pour le mode réel.

Dans un terminal ouvert dans `INF3995-106/project_ws` :
```bash
colcon build
source install/setup.sh
ros2 launch robot_exploration robot_bringup.launch.py
```

---

## Étape 4 : Application Web (sur votre ordinateur)

### 4.1 Lancer le serveur (avec connexion ROS2)
Depuis le dossier `project_ws` :
```bash
colcon build
source install/setup.sh
cd ../server
npm rebuild rclnodejs
npm start
```

### 4.2 Lancer le client
Dans le dossier `client` :
```bash
npm start
```

---

## Nettoyage d’un ancien build (si nécessaire)
Dans le dossier `project_ws` :
```bash
rm -rf build log install
```
> Ignorez les **Warnings** lors de la reconstruction.

---

# 📘 Conventions de codage et de structure du projet

Afin d’assurer la cohérence, la lisibilité et la maintenabilité du code, l’équipe a défini un ensemble de conventions communes appliquées à l’ensemble du projet.  
Ces règles couvrent la **structure**, le **nommage** et le **formatage** du code dans les différentes composantes du système.

---

## 🧩 1. Structure du projet

Le projet est organisé selon une architecture modulaire en trois sections principales :

project_root/
│
├── client/ → Interface web (Angular)
│ └── services/, components/, assets/
│
├── server/ → Backend (NestJS)
│ └── services/, controllers/, gateways/
│
└── project_ws/ → Partie embarquée (ROS2)
└── src/, launch/, config/, maps/

markdown
Copier le code

Chaque section est structurée en **packages** et **services** afin d’assurer la modularité du code.  
Cette organisation facilite le travail parallèle des membres de l’équipe, la maintenance et le déploiement indépendant de chaque couche.

---

## 🧠 2. Conventions de nommage

### 🐍 Partie embarquée (`project_ws`)
Les conventions suivent les standards Python et ROS 2 :
- **`snake_case`** → pour les **variables**, **fonctions** et **méthodes**  
  _Exemples :_ `update_map_cost()`, `current_goal`, `get_robot_pose()`
- **`PascalCase`** → pour les **classes**  
  _Exemples :_ `MissionServer`, `ExplorerNode`

### 🌐 Partie web (`client` et `server`)
Les conventions respectent les standards du développement TypeScript :
- **`camelCase`** → pour les **variables**, **méthodes** et **fonctions**  
  _Exemples :_ `startMission()`, `robotState`, `updatePosition()`
- **`PascalCase`** → pour les **classes**, **services** et **composants**  
  _Exemples :_ `MissionService`, `SocketService`, `MapComponent`

---

> 🧭 Ces conventions garantissent un code propre, uniforme et facile à maintenir, quel que soit le module du projet (embarqué, serveur ou client).

---
