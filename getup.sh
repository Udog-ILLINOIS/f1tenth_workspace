#!/usr/bin/env bash
# Get the sim up: club stack container, bridge, AutoDRIVE (connected), the track's saved map in the
# Mapping tab. Then ./go.sh (another terminal) races.
#   ./getup.sh [TRACK]     porto by default; a running sim is switched to TRACK
# Switching track in the sim afterwards loads that track's map (autodrive/map_feed.py, from
# planners/sigrobotics/ws/tracks/<track>/). Ctrl-C stops the map feed; the container, bridge and sim stay up.
set -euo pipefail
root="$(cd "$(dirname "$0")" && pwd)"
track=${1:-porto}
compose=(docker compose -f "$root/planners/sigrobotics/docker-compose.yml")

if docker ps --format '{{.Names}}' | grep -qi forzaeth; then
  echo "== stopping ForzaETH (port 4567)"; "$root/forzaeth.sh" down
fi

echo "== club stack container"
"${compose[@]}" up -d

if "${compose[@]}" exec -T ros pgrep -f '^/usr/bin/python3 /root/ws/install/autodrive_bridge/lib/autodrive_bridge/fast_bridge' >/dev/null; then
  echo "== bridge already running"
else
  echo "== starting bridge (log: /tmp/bridge.log in the container)"
  "${compose[@]}" exec -d ros bash -c "source /opt/ros/humble/setup.bash; source /root/ws/install/setup.bash; \
    exec ros2 launch autodrive_bridge autodrive_launch.py > /tmp/bridge.log 2>&1"
fi

if "$root/autodrive.sh" ctl status >/dev/null 2>&1; then
  echo "== sim already running, switching to $track"
  "$root/autodrive.sh" ctl track "$track" >/dev/null
else
  echo "== opening AutoDRIVE on $track"
  "$root/autodrive.sh" sim --track "$track"
fi

connected() { "$root/autodrive.sh" ctl status 2>/dev/null | grep -q '"connected":true'; }
echo -n "== waiting for the sim to connect to the bridge"
for _ in $(seq 90); do connected && break; echo -n .; sleep 1; done
if connected; then
  echo; echo "== CONNECTED: sim <-> bridge on 127.0.0.1:4567 ($("$root/autodrive.sh" ctl status | grep -o '"track":"[^"]*"'))"
else
  echo; echo "sim did not connect to the bridge in 90 s (see /tmp/bridge.log in the container)" >&2; exit 1
fi

echo "== map feed (Ctrl-C to stop); race with ./go.sh"
exec python3 "$root/autodrive/map_feed.py"
