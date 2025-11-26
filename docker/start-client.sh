#!/usr/bin/env bash
set -euo pipefail

APP_ENV="${APP_ENV:-development}"
NODE_ENV="${NODE_ENV:-$APP_ENV}"
CLIENT_PORT="${CLIENT_PORT:-4200}"
export APP_ENV NODE_ENV CLIENT_PORT

DIST_DIR="dist/client"
if [ -d "${DIST_DIR}/browser" ]; then
    DIST_DIR="${DIST_DIR}/browser"
fi

if [ "${APP_ENV}" = "production" ]; then
    exec http-server "${DIST_DIR}" -a 0.0.0.0 -p "${CLIENT_PORT}"
else
    exec npm exec ng serve -- --host 0.0.0.0 --port "${CLIENT_PORT}"
fi
