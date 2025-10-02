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

