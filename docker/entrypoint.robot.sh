#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/"${ROS_DISTRO}"/setup.bash
cd /ws

LOCK_FILE=/ws/.build_lock
BUILD_READY=/ws/.build_ready

# Fonction pour acquérir le lock
acquire_lock() {
  max_attempts=30
  attempt=0
  while [ -f "$LOCK_FILE" ] && [ $attempt -lt $max_attempts ]; do
    echo "Waiting for build lock to be released... (attempt $attempt/$max_attempts)";
    sleep 2;
    attempt=$((attempt + 1));
  done;
  
  if [ -f "$LOCK_FILE" ]; then
    echo "ERROR: Could not acquire build lock after $max_attempts attempts";
    exit 1;
  fi;
  
  touch "$LOCK_FILE";
  echo "Build lock acquired";
}

# Si le workspace est déjà prêt, ne pas rebuilder
if [ -f "$BUILD_READY" ] && [ -f install/setup.bash ] && ([ -d install/share/limo_interfaces ] || [ -L install/limo_interfaces ] || [ -d install/limo_interfaces ]); then
  echo 'Workspace already built, skipping build';
else
  # Nettoyer AVANT d'acquérir le lock pour éviter les conflits de chemins
  echo 'Cleaning old build artifacts before acquiring lock...';
  rm -rf build install log "$BUILD_READY"
  
  # Acquérir le lock avec retry
  acquire_lock;
  trap 'rm -f $LOCK_FILE' EXIT;
  
  # Attendre un peu pour s'assurer que l'autre processus voit le lock
  sleep 1;
  
  # Vérifier à nouveau si le workspace est prêt (peut-être que l'autre processus vient de finir)
  if [ -f "$BUILD_READY" ] && [ -f install/setup.bash ] && ([ -d install/share/limo_interfaces ] || [ -L install/limo_interfaces ] || [ -d install/limo_interfaces ]); then
    echo 'Workspace now ready (built by another process), releasing lock';
    rm -f "$LOCK_FILE";
  else
    # Créer les répertoires nécessaires AVANT le build
    mkdir -p log build install

    echo 'Building ROS2 workspace...';
    colcon build --merge-install --symlink-install

    # Attendre un peu pour que les fichiers soient écrits
    sleep 2;
    
    if [ -f install/setup.bash ] && ([ -d install/share/limo_interfaces ] || [ -L install/limo_interfaces ] || [ -d install/limo_interfaces ]); then
      source install/setup.bash
      touch "$BUILD_READY";
      echo 'ROS2 workspace built successfully!'
    else
      echo 'ERROR: Build failed - install/setup.bash or limo_interfaces not found'
      echo 'Checking install directory:';
      ls -la install/ 2>&1 || echo 'install directory does not exist';
      echo 'Checking for limo_interfaces:';
      ls -la install/limo_interfaces 2>&1 || echo 'limo_interfaces not found';
      ls -la install/share/limo_interfaces 2>&1 || echo 'install/share/limo_interfaces not found';
      rm -f "$LOCK_FILE";
      exit 1
    fi
    
    # Libérer le lock
    rm -f "$LOCK_FILE";
  fi
fi

# Garder le conteneur en vie pour que le serveur puisse utiliser les fichiers
exec bash

