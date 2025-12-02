#!/bin/bash

# ============================================================
# Script unique pour démarrer ROS2 pour LIMO1 ou LIMO2
# Usage :
#    ./start_robot.sh          → lance limo1
#    ./start_robot.sh limo2    → lance limo2
# ============================================================

# Déterminer le namespace ROS passé en argument (default = limo1)
NAMESPACE="$1"

if [ -z "$NAMESPACE" ]; then
    NAMESPACE="limo1"
fi

echo "------------------------------------------------"
echo "   Lancement ROS avec namespace : $NAMESPACE"
echo "------------------------------------------------"

# --- 1. Donner accès au port série (nécessaire après chaque boot) ---
echo "→ Configuration du port série..."
sudo chmod 666 /dev/ttyTHS1

# --- 2. Aller dans le workspace ---
WORKSPACE=project_ws/

echo "→ Passage dans $WORKSPACE"
cd "$WORKSPACE" || { echo "Workspace introuvable"; exit 1; }

# --- 3. Nettoyage complet (comme tu le fais) ---
echo "→ Nettoyage build/ install/ log/"
rm -rf build/ install/ log/

# --- 4. Compilation ---
echo "→ Compilation colcon..."
colcon build --symlink-install

# --- 5. Sourcing ---
echo "→ Source setup.sh..."
source install/setup.sh

# --- 6. Démarrage du bringup ---
echo "→ Démarrage du bringup pour $NAMESPACE..."

# Si limo1 → pas de namespace dans ton launch
if [ "$NAMESPACE" == "limo1" ]; then
    ros2 launch robot_exploration robot_bringup.launch.py
else
    ros2 launch robot_exploration robot_bringup.launch.py namespace:="$NAMESPACE"
fi
