#!/bin/bash

# ============================================================
# Script UNIVERSAL Zenoh RMW
# Fonctionne sur ROBOTS et PC
# Usage : ./start_zenoh.sh <host1> [host2] ...
# ============================================================

if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <zenoh_endpoint_1> [zenoh_endpoint_2] ..."
    exit 1
fi

# Construire la liste d'endpoints à partir des arguments
ENDPOINTS=""
for host in "$@"; do
    ENDPOINTS="${ENDPOINTS}\"tcp/${host}:7447\", "
done

# Enlever la virgule finale
ENDPOINTS="[${ENDPOINTS::-2}]"

# Exports obligatoires
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE="connect/endpoints=${ENDPOINTS}"

echo "=============================================="
echo "Lancement Zenoh RMW"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "ZENOH_CONFIG_OVERRIDE=$ZENOH_CONFIG_OVERRIDE"
echo "=============================================="

# Lancer rmw_zenohd
ros2 run rmw_zenoh_cpp rmw_zenohd
