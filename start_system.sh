#!/bin/bash

# ============================================================
# Script complet : Zenoh + Build + ROS2 ou Restart rapide
#
# Modes :
#   FULL (par défaut) → Zenoh + build + bringup
#   RESTART           → seulement restart du bringup ROS
#
# Usage :
#   ./start_system.sh [full|restart] <namespace> <zenoh_host1> [zenoh_host2...]
#
# Exemples :
#   FULL :    ./start_system.sh limo1 lm1335.local lm1367.local
#   FULL :    ./start_system.sh full limo2 192.168.0.41
#   RESTART : ./start_system.sh restart limo1
# ============================================================


# ===========================
# 0. Parsing des arguments
# ===========================

MODE="full"
FIRST_ARG="$1"

# Si le premier argument est le mode → on le prend
if [[ "$FIRST_ARG" == "full" || "$FIRST_ARG" == "restart" ]]; then
    MODE="$1"
    shift
fi

# Namespace obligatoire
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 [full|restart] <namespace> [zenoh_hosts...]"
    exit 1
fi

NAMESPACE="$1"
shift  # retire namespace

echo "------------------------------------------------"
echo "   Mode : $MODE"
echo "   Namespace ROS : $NAMESPACE"
echo "------------------------------------------------"


# ============================================================
# MODE RESTART → ne lance PAS Zenoh, ne build PAS
# ============================================================
if [ "$MODE" == "restart" ]; then

    echo "→ Restart rapide du bringup ROS2"
    cd project_ws/ || { echo "project_ws introuvable"; exit 1; }

    echo "→ Source de l’environnement existant..."
    source install/setup.sh

    echo "→ Démarrage du bringup..."

    if [ "$NAMESPACE" == "limo1" ]; then
        ros2 launch robot_exploration robot_bringup.launch.py
    else
        ros2 launch robot_exploration robot_bringup.launch.py namespace:="$NAMESPACE"
    fi

    exit 0
fi


# ============================================================
# MODE FULL → Zenoh + build + ROS2
# ============================================================
echo "→ Lancement complet du système (FULL mode)"


# =======================
# 1. CONFIG ZENOH
# =======================
echo "→ Construction de la config Zenoh..."

ENDPOINTS=""
for host in "$@"; do
    ENDPOINTS="${ENDPOINTS}\"tcp/${host}:7447\", "
done
ENDPOINTS="[${ENDPOINTS::-2}]"

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE="connect/endpoints=${ENDPOINTS}"

echo "   RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "   ZENOH_CONFIG_OVERRIDE=$ZENOH_CONFIG_OVERRIDE"


# =======================
# 2. LANCEMENT ZENOH (process séparé)
# =======================
echo "→ Démarrage de Zenoh en tâche de fond..."
nohup ros2 run rmw_zenoh_cpp rmw_zenohd > ~/zenoh.log 2>&1 &
sleep 1


# =======================
# 3. BUILD ROS2
# =======================
echo "→ Configuration du port série..."
sudo chmod 666 /dev/ttyTHS1

WORKSPACE=project_ws/
echo "→ Passage dans $WORKSPACE"
cd "$WORKSPACE" || { echo "project_ws introuvable"; exit 1; }

echo "→ Nettoyage build/ install/ log/"
rm -rf build/ install/ log/

echo "→ Compilation colcon..."
colcon build --symlink-install

echo "→ Source setup.sh..."
source install/setup.sh


# =======================
# 4. LANCEMENT DU BRINGUP
# =======================
echo "→ Lancement du bringup ROS2 pour : $NAMESPACE"

if [ "$NAMESPACE" == "limo1" ]; then
    ros2 launch robot_exploration robot_bringup.launch.py
else
    ros2 launch robot_exploration robot_bringup.launch.py namespace:="$NAMESPACE"
fi
