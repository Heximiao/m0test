#!/usr/bin/env bash
set -eo pipefail

for pattern in \
  '^ros-jazzy-nav2-' \
  '^ros-jazzy-.*amcl' \
  '^ros-jazzy-.*map-server' \
  '^ros-jazzy-slam-toolbox' \
  '^ros-jazzy-cartographer' \
  '^ros-jazzy-navigation2'
do
  echo "=== ${pattern} ==="
  apt-cache search "${pattern}" || true
done
