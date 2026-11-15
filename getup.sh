#!/usr/bin/env bash
# Get the sim up: club stack container, bridge, AutoDRIVE (connected), the track's saved map in the
# Mapping tab. Then ./go.sh (another terminal) races.
#   ./getup.sh [TRACK]     porto by default; a running sim is switched to TRACK
#   ./getup.sh --driver 2  driver 2's container and bridge (port 4568, ROS domain 2) for car 2 of a sim
#                          started with --cars 2 or more (started so if it is not running); no map feed
# Switching track in the sim afterwards loads that track's map (autodrive/map_feed.py, from
# planners/sigrobotics/ws/tracks/<track>/). Ctrl-C stops the map feed; the container, bridge and sim stay up.
# The bridge runs with use_sim_time:=true: it publishes the sim's time on /clock and the stack (go.sh) runs on
# it, so a sim freeze stops the stack's clock too. After restarting the sim app (its time starts at 0 again)
# restart the bridge and the race (Ctrl-C go.sh; stop the bridge; ./getup.sh; ./go.sh).
set -euo pipefail
root="$(cd "$(dirname "$0")" && pwd)"
driver=1
if [ "${1:-}" = "--driver" ]; then driver=$2; shift 2; fi
track=${1:-porto}
source "$root/autodrive/driver_env.sh" "$driver"   # compose=(...) for this driver's container, car_connected

start_bridge() {   # this driver's bridge in its container, unless already running
  if "${compose[@]}" exec -T ros pgrep -f '^/usr/bin/python3 /root/ws/install/autodrive_bridge/lib/autodrive_bridge/fast_bridge' >/dev/null; then
    echo "== bridge already running"
    "${compose[@]}" exec -T ros bash -c "source /opt/ros/humble/setup.bash; ros2 param get /fast_bridge use_sim_time" \
        2>/dev/null | grep -q True \
      || echo "== WARNING: the running bridge is not on the sim's clock (started without use_sim_time:=true):" \
              "stop it (Ctrl-C / kill -INT its ros2 launch) and run getup.sh again" >&2
  else
    echo "== starting bridge (log: /tmp/bridge.log in the container)"
    "${compose[@]}" exec -d ros bash -c "source /opt/ros/humble/setup.bash; source /root/ws/install/setup.bash; \
      exec ros2 launch autodrive_bridge autodrive_launch.py use_sim_time:=true > /tmp/bridge.log 2>&1"
  fi
}
wait_connected() {   # car $driver connected to its bridge within 90 s (car_connected: autodrive/driver_env.sh)
  echo -n "== waiting for car $driver to connect to its bridge"
  for _ in $(seq 90); do car_connected && { echo; return 0; }; echo -n .; sleep 1; done
  echo; return 1
}

if [ "$driver" = 1 ] && docker ps --format '{{.Names}}' | grep -qi forzaeth; then
  echo "== stopping ForzaETH (port 4567)"; "$root/forzaeth.sh" down
fi

if [ "$driver" = 1 ]; then echo "== club stack container"
else echo "== driver $driver container (bridge on host port $BRIDGE_PORT, ROS domain $ROS_DOMAIN_ID)"; fi
"${compose[@]}" up -d
start_bridge

if "$root/autodrive.sh" ctl status >/dev/null 2>&1; then
  if [ "$driver" = 1 ]; then
    echo "== sim already running, switching to $track"
    "$root/autodrive.sh" ctl track "$track" >/dev/null
  fi
else
  echo "== opening AutoDRIVE on $track"
  if [ "$driver" = 1 ]; then "$root/autodrive.sh" sim --track "$track"
  else "$root/autodrive.sh" sim --track "$track" --cars "$driver"; fi
fi

if ! wait_connected; then
  if [ "$driver" = 1 ]; then echo "sim did not connect to the bridge in 90 s (see /tmp/bridge.log in the container)" >&2
  else echo "car $driver did not connect in 90 s (is the sim running with --cars $driver or more? ./autodrive.sh ctl cars)" >&2; fi
  exit 1
fi
if [ "$driver" != 1 ]; then
  echo "== CONNECTED: car $driver <-> bridge on 127.0.0.1:$BRIDGE_PORT; race with ./go.sh --driver $driver"
  exit 0
fi
echo "== CONNECTED: sim <-> bridge on 127.0.0.1:4567 ($("$root/autodrive.sh" ctl status | grep -o '"track":"[^"]*"'))"

echo "== map feed (Ctrl-C to stop); race with ./go.sh"
exec python3 "$root/autodrive/map_feed.py"
