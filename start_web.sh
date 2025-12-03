#!/bin/bash
set -e

# ============================================================
# Script web : Zenoh + build + serveur + client (sans tmux)
# Reproduit la séquence manuelle :
#   1) Zenoh en tâche de fond
#   2) colcon build dans project_ws
#   3) export RMW_IMPLEMENTATION
#   4) source install/setup.sh
#   5) npm start dans server
#   6) detection adresse IP
#   7) ng serve dans client
#
# Usage :
#   ./start_web.sh <zenoh_host1> [zenoh_host2 ...]
# ============================================================

cd "$(dirname "$0")"

if [ "$#" -lt 1 ]; then
  echo "Aucun endpoint Zenoh fourni."
  echo "Usage : $0 <zenoh_host1> [zenoh_host2 ...]"
  exit 1
fi

PIDS=()

USER_IP=$(hostname -I | awk '{print $1}')
echo "→ IP détectée : $USER_IP"

ENV_DIR="./client/src/environments"
ENV_FILE="$ENV_DIR/environment.ts"
ENV_DYNAMIC="$ENV_DIR/environment.prod.ts"

echo "→ Génération du environment.prod.ts..."

cat <<EOF > "$ENV_DYNAMIC"
export const environment = {
    production: false,
    serverUrl: 'http://$USER_IP:3000',
};
EOF

cp "$ENV_DYNAMIC" "$ENV_FILE"

echo "→ environment.ts mis à jour : http://$USER_IP:3000"
echo

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

echo "→ Démarrage de Zenoh en tâche de fond..."
nohup ros2 run rmw_zenoh_cpp rmw_zenohd > ~/zenoh.log 2>&1 &
ZENOH_PID=$!
PIDS+=("$ZENOH_PID")
sleep 1

echo "→ Build du workspace ROS2..."
cd project_ws || { echo "project_ws introuvable"; exit 1; }
colcon build

echo "→ Source de l’environnement ROS2..."
source install/setup.sh

echo "→ Lancement du serveur..."
cd ../server || { echo "server introuvable"; exit 1; }
npm start &
SERVER_PID=$!
PIDS+=("$SERVER_PID")

echo "→ Lancement du client..."
cd ../client || { echo "client introuvable"; exit 1; }
ng serve --host 0.0.0.0 &
CLIENT_PID=$!
PIDS+=("$CLIENT_PID")

echo "Tout est lancé !"
echo "Client PID : $CLIENT_PID"
echo "Server PID : $SERVER_PID"
echo "Zenoh PID  : $ZENOH_PID"
echo
echo "Appuie sur Ctrl+C pour arrêter proprement."

cleanup() {
  echo "Arrêt..."

  for pid in "${PIDS[@]}"; do
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done

  sleep 1

  for pid in "${PIDS[@]}"; do
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      kill -9 "$pid" 2>/dev/null || true
    fi
  done

  wait "${PIDS[@]}" 2>/dev/null || true
  exit 0
}

trap cleanup SIGINT SIGTERM

wait "${PIDS[@]}"
