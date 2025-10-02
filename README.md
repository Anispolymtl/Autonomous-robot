# 🚀 Guide Général — Démarrage Robot & Application Web

Ce guide regroupe **toutes les étapes** nécessaires pour démarrer les robots et l’application web, de manière **simple et uniforme**.

---

## 1) Connexion au robot (dans chaque terminal)
Exécutez ces étapes **dans chaque terminal** avant de lancer un robot :
```bash
ssh -X [votre_user]@lmXXX.local
# Mot de passe (quand demandé) : h2025!

sudo chmod 666 /dev/ttyTHS1
```
> Remplacez `lmXXX.local` par l’identifiant de votre robot (ex. lm1335.local, lm1166local, etc.).  

---

## 2) Démarrage des robots (deux terminaux séparés)

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

## 3) Application Web (sur votre ordinateur)

### 3.1 Lancer le serveur (avec connexion ROS2)
Depuis le dossier `project_ws` :
```bash
colcon build
source install/setup.sh
cd ../server
npm rebuild rclnodejs
npm start
```

### 3.2 Lancer le client
Dans le dossier `client` :
```bash
npm start
```

---

## 4) Nettoyage d’un ancien build (si nécessaire)
Dans le dossier `project_ws` :
```bash
rm -rf build log install
```
> Ignorez les **Warnings** lors de la reconstruction.

---

✅ C’est tout ! Suivez ces étapes dans l’ordre : **connexion → build + lancement des robots → serveur → client**.