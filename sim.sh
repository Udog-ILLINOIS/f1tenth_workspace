#!/usr/bin/env bash
# Run docker-compose for the Foxy or Humble sim stack, from anywhere.
#   ./sim.sh foxy up -d        ./sim.sh foxy stop / start / down / ps
#   ./sim.sh humble up -d --build   (first Humble run builds the image)
# Refuses to start a stack while f1tenth_gym_ros is on the wrong branch, or while the other
# stack is still running (both publish noVNC on port 8080).
set -euo pipefail
root="$(cd "$(dirname "$0")" && pwd)"
distro="${1:-}"; shift || true
case "$distro" in
  foxy)   branch=main;      project=f1tenth_gym_ros_foxy; override="$root/docker/foxy_labs.compose.yml" ;;
  humble) branch=dev-humble; project=f1tenth_gym_ros_humble; override="$root/docker/humble.compose.yml" ;;
  *) echo "usage: $0 <foxy|humble> <docker-compose args...>" >&2; exit 2 ;;
esac
[ $# -gt 0 ] || { echo "usage: $0 $distro <docker-compose args...>" >&2; exit 2; }

needs_guard=0
for a in "$@"; do
  case "$a" in up|start|restart|create|run|build) needs_guard=1 ;; esac
done
case "$needs_guard" in
  1)
    cur="$(git -C "$root/gym_ros_workspace/f1tenth_gym_ros" rev-parse --abbrev-ref HEAD)"
    if [ "$cur" != "$branch" ]; then
      echo "f1tenth_gym_ros is on '$cur', but the $distro stack needs '$branch'." >&2
      echo "Run: git -C '$root/gym_ros_workspace/f1tenth_gym_ros' checkout $branch" >&2
      exit 1
    fi
    other="$(docker ps --filter publish=8080 --format '{{.Names}}' | grep -v "^${project}-" || true)"
    if [ -n "$other" ]; then
      echo "Port 8080 is in use by: $other" >&2
      echo "Stop the other stack first (e.g. ./sim.sh <other> stop)." >&2
      exit 1
    fi ;;
esac

# Humble's noVNC service sits behind a compose profile; commands that manage existing containers
# must include it or a running noVNC container is silently left behind (holding port 8080).
profile=()
if [ "$distro" = humble ]; then
  for a in "$@"; do
    case "$a" in stop|down|ps|logs) profile=(--profile novnc) ;; esac
  done
fi

cd "$root/gym_ros_workspace/f1tenth_gym_ros"
exec docker-compose -p "$project" ${profile[@]+"${profile[@]}"} -f docker-compose.yml -f "$override" "$@"
