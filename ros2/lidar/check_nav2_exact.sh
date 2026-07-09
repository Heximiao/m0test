#!/usr/bin/env bash
set -eo pipefail

for pattern in \
  '^ros-lyrical-nav2-' \
  '^ros-lyrical-.*amcl' \
  '^ros-lyrical-.*map-server' \
  '^ros-lyrical-slam-toolbox' \
  '^ros-lyrical-cartographer' \
  '^ros-lyrical-navigation2'
do
  echo "=== ${pattern} ==="
  apt-cache search "${pattern}" || true
done
