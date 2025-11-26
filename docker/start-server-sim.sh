#!/usr/bin/env bash
set -euo pipefail

APP_ENV="${APP_ENV:-development}"
NODE_ENV="${NODE_ENV:-$APP_ENV}"
PORT="${PORT:-3000}"
export APP_ENV NODE_ENV PORT

ROS_DISTRO="${ROS_DISTRO:-humble}"
WS="${WS:-/project_ws}"
export RCLNODEJS_ROS_DISTRO="${ROS_DISTRO}"
export RCLNODEJS_ROS_VERSION=2
export RCUTILS_LOGGING_SEVERITY_THRESHOLD="${RCUTILS_LOGGING_SEVERITY_THRESHOLD:-ERROR}"
export RCL_LOG_LEVEL="${RCL_LOG_LEVEL:-error}"
export ROS_PACKAGE_PATH="${WS}/install:${ROS_PACKAGE_PATH:-}"
# Avoid unbound variables when sourcing ROS setup files with `set -u`
: "${AMENT_TRACE_SETUP_FILES:=}"
: "${AMENT_PYTHON_EXECUTABLE:=$(command -v python3)}"
: "${COLCON_TRACE:=0}"
: "${COLCON_PREFIX_PATH:=}"
: "${COLCON_PYTHON_EXECUTABLE:=$(command -v python3)}"
: "${CMAKE_PREFIX_PATH:=}"
: "${AMENT_PREFIX_PATH:=}"
: "${LD_LIBRARY_PATH:=}"
: "${PYTHONPATH:=}"
: "${IGN_GAZEBO_RESOURCE_PATH:=}"
: "${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:=}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -f "${WS}/install/setup.bash" ]; then
    # Use existing workspace artifacts when present
    # shellcheck disable=SC1090
    source "${WS}/install/setup.bash"
else
    echo "No built workspace detected under ${WS}, running colcon build..."
    cd "${WS}"
    colcon build --merge-install --symlink-install
    # shellcheck disable=SC1090
    source "${WS}/install/setup.bash"
fi

cd "${WS}"
ros2 launch simulation_bringup diff_drive.launch.py >> /tmp/ros_launch.log 2>&1 &
SIM_PID=$!

cd /app
# Locate server build output (different Nest configs use different out dirs)
resolve_server_main() {
    # Restrict lookup to build outputs we expect from Nest (avoid picking node_modules main.js)
    for path in \
        "/app/out/server/app/index.js" \
        "/app/out/index.js" \
        "/app/dist/server/app/index.js" \
        "/app/dist/main.js"
    do
        if [ -f "${path}" ]; then
            echo "${path}"
            return
        fi
    done
    local found=""
    found=$(find /app/out -maxdepth 5 -type f -name "index.js" 2>/dev/null | head -n 1 || true)
    if [ -z "${found}" ]; then
        found=$(find /app/dist -maxdepth 5 -type f \( -name "main.js" -o -name "index.js" \) 2>/dev/null | head -n 1 || true)
    fi
    echo "${found}"
}

SERVER_MAIN="$(resolve_server_main)"

if [ -z "${SERVER_MAIN}" ]; then
    echo "Server build output not found. Attempting to build..." >&2
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    source "${WS}/install/setup.bash"
    npm run build
    SERVER_MAIN="$(resolve_server_main)"
    if [ -z "${SERVER_MAIN}" ]; then
        echo "Server build output still missing after build (searched under /app/out and /app/dist)." >&2
        exit 1
    fi
fi

# Regenerate rclnodejs interfaces against the current ROS workspace (covers limo_interfaces)
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WS}/install/setup.bash"
npm rebuild rclnodejs --build-from-source
if [ ! -d "/app/node_modules/rclnodejs/generated/limo_interfaces" ]; then
    echo "limo_interfaces not generated. Available generated packages:" >&2
    ls -1 /app/node_modules/rclnodejs/generated 2>/dev/null >&2 || true
    echo "ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}" >&2
    exit 1
fi

echo "Starting server from ${SERVER_MAIN}"
node "${SERVER_MAIN}" &
SERVER_PID=$!

cleanup() {
    kill "${SERVER_PID}" 2>/dev/null || true
    kill "${SIM_PID}" 2>/dev/null || true
    wait "${SERVER_PID}" 2>/dev/null || true
    wait "${SIM_PID}" 2>/dev/null || true
}

trap cleanup SIGINT SIGTERM
wait -n "${SERVER_PID}" "${SIM_PID}"
cleanup
