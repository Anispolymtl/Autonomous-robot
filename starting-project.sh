#!/bin/bash
set -e

cd "$(dirname "$0")"

USER_IP=$(hostname -I | awk '{print $1}')
echo "---- IP détectée : $USER_IP ----"

ENV_DIR="./client/src/environments"
ENV_FILE="$ENV_DIR/environment.ts"
ENV_DYNAMIC="$ENV_DIR/environment.prod.ts"

echo "---- Génération du environment.prod.ts ----"

cat <<EOF > "$ENV_DYNAMIC"
export const environment = {
    production: false,
    serverUrl: 'http://$USER_IP:3000',
};
EOF

cp "$ENV_DYNAMIC" "$ENV_FILE"

echo "environment.ts mis à jour avec : http://$USER_IP:3000"
echo

echo "---- Build du workspace ROS2 ----"
cd project_ws
colcon build
source install/setup.sh

echo "---- Lancement du client ----"
cd ../client
npx ng serve --host 0.0.0.0 &
CLIENT_PID=$!

echo "---- Lancement du serveur ----"
cd ../server
npm rebuild rclnodejs
npm start &
SERVER_PID=$!

echo "---- Lancement de la simulation ----"
cd ../project_ws
ros2 launch simulation_bringup diff_drive.launch.py &
ROS2_PID=$!

echo "Tout est lancé !"
echo "Client PID  : $CLIENT_PID"
echo "Server PID  : $SERVER_PID"
echo "ROS2 PID    : $ROS2_PID"
echo
echo "Appuie sur Ctrl+C pour tout arrêter."

cleanup() {
  echo "Arrêt..."

  for pid in "$CLIENT_PID" "$SERVER_PID" "$ROS2_PID"; do
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done

  sleep 1

  for pid in "$CLIENT_PID" "$SERVER_PID" "$ROS2_PID"; do
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      kill -9 "$pid" 2>/dev/null || true
    fi
  done

  wait "$CLIENT_PID" "$SERVER_PID" "$ROS2_PID" 2>/dev/null || true
  exit 0
}

trap cleanup SIGINT SIGTERM

wait "$CLIENT_PID" "$SERVER_PID" "$ROS2_PID"
