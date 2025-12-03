#!/bin/bash
set -e

# Usage :
#   ./starting-project.sh --simulation
#   ./starting-project.sh <zenoh_host1> [zenoh_host2 ...]
#   - Sans option → démarre Zenoh + serveur + client (au moins 1 host Zenoh requis)
#   - --simulation → démarre la simulation (ROS2) + serveur + client, sans Zenoh

cd "$(dirname "$0")"

SIMULATION=false
ZENOH_HOSTS=()

for arg in "$@"; do
  case "$arg" in
    -s|--simulation)
      SIMULATION=true
      ;;
    -h|--help)
      echo "Usage: $0 --simulation | <zenoh_host1> [zenoh_host2 ...]"
      exit 0
      ;;
    *)
      ZENOH_HOSTS+=("$arg")
      ;;
  esac
done

PIDS=()

if ! $SIMULATION; then
  if [ "${#ZENOH_HOSTS[@]}" -eq 0 ]; then
    echo "Aucun endpoint Zenoh fourni."
    echo "Usage: $0 --simulation | <zenoh_host1> [zenoh_host2 ...]"
    exit 1
  fi

  ENDPOINTS=""
  for host in "${ZENOH_HOSTS[@]}"; do
    ENDPOINTS="${ENDPOINTS}\"tcp/${host}:7447\", "
  done
  ENDPOINTS="[${ENDPOINTS::-2}]"

  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  export ZENOH_CONFIG_OVERRIDE="connect/endpoints=${ENDPOINTS}"

  echo "---- Lancement de Zenoh (rmw_zenohd) ----"
  ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/zenoh.log 2>&1 &
  ZENOH_PID=$!
  PIDS+=("$ZENOH_PID")
  sleep 1
else
  echo "---- Mode simulation : Zenoh désactivé ----"
fi

echo "---- Build du workspace ROS2 ----"
cd project_ws
colcon build
source install/setup.sh


if $SIMULATION; then
  echo "---- Lancement de la simulation ----"
  cd ../project_ws
  ros2 launch simulation_bringup diff_drive.launch.py &
  ROS2_PID=$!
  PIDS+=("$ROS2_PID")
fi

echo "---- Lancement du serveur ----"
cd ../server
npm rebuild rclnodejs
npm start &
SERVER_PID=$!
PIDS+=("$SERVER_PID")

echo "---- Lancement du client ----"
cd ../client
npm start &
CLIENT_PID=$!
PIDS+=("$CLIENT_PID")

echo "Tout est lancé !"
echo "Client PID   : $CLIENT_PID"
echo "Server PID   : $SERVER_PID"
if $SIMULATION; then
  echo "ROS2 PID     : $ROS2_PID"
else
  echo "Zenoh PID    : $ZENOH_PID"
fi
echo
echo "Appuie sur Ctrl+C pour tout arrêter."

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
