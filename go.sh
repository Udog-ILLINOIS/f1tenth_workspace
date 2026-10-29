#!/usr/bin/env bash
# Race the sim's current track: club stack race.sh with MCL localization and obstacle avoidance.
#   ./go.sh [TRACK] [LAPS] [race.sh args...]   TRACK: the sim's current track by default
#   ./go.sh                    race until Ctrl-C
#   ./go.sh 5                  stop the car after 5 laps
#   ./go.sh berlin 3 speed:=0.7                defaults: speed:=0.5 localizer:=mcl avoid:=true
#   speed:=S is race.sh's speed_scale:=S (drive speed = raceline vx * S)
# Needs ./getup.sh first (container, bridge, connected sim). Ctrl-C (or the last lap) stops the race
# nodes; the bridge then zeroes the car's commands (fast_bridge cmd_timeout) and waits until it stands.
set -euo pipefail
root="$(cd "$(dirname "$0")" && pwd)"
compose=(docker compose -f "$root/planners/sigrobotics/docker-compose.yml")
in_ros() { "${compose[@]}" exec -T ros bash -c "source /opt/ros/humble/setup.bash; $1"; }

"$root/autodrive.sh" ctl status 2>/dev/null | grep -q '"connected":true' \
  || { echo "sim not connected to a bridge: run ./getup.sh first" >&2; exit 1; }

track=; laps=
if [ $# -gt 0 ] && [[ "$1" != *:=* && ! "$1" =~ ^[0-9]+$ ]]; then track=$1; shift; fi
if [ $# -gt 0 ] && [[ "$1" =~ ^[0-9]+$ ]]; then laps=$1; shift; fi
sim_track=$(python3 "$root/autodrive/map_feed.py" --current)
[ -n "$track" ] || track=$sim_track
tracks="$root/planners/sigrobotics/ws/tracks"
if [ ! -f "$tracks/$track/raceline.csv" ]; then
  echo "no mapped track '$track' (tracks with a raceline: $(cd "$tracks" && ls */raceline.csv | sed 's|/raceline.csv||' | tr '\n' ' '))" >&2
  trap - EXIT; exit 1
fi
if [ "$track" != "$sim_track" ]; then   # race.sh drives this track's raceline: put the sim on it too
  base=$track
  until "$root/autodrive.sh" ctl tracks | grep -q "\"name\":\"$base\""; do   # porto_v2 -> porto
    [[ "$base" == *_* ]] || { echo "the sim has no track '$track' (sim is on $sim_track)" >&2; exit 1; }
    base=${base%_*}
  done
  if [ "$base" != "$sim_track" ]; then
    echo "== switching the sim from $sim_track to $base"
    "$root/autodrive.sh" ctl track "$base" >/dev/null; sleep 3
  fi
fi

race_nodes='^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch control race.launch.py'
# SIGINT to the launch process alone reaches no node (they get SIGTERM only 5 s later, still driving),
# so signal its nodes (its children) as well
kill_race="for p in \$(pgrep -f '$race_nodes'); do pkill -INT -P \$p; kill -INT \$p; done"
speed() {   # |v| of the sim car [m/s], from the bridge's ground truth odometry
  in_ros "timeout 5 ros2 topic echo --once --field twist.twist.linear /ground_truth/odom 2>/dev/null" \
    | python3 -c 'import sys,re; v=[float(x) for x in re.findall(r"[xyz]: (\S+)", sys.stdin.read())]; print("%.2f" % sum(a*a for a in v)**.5 if v else "?")'
}
stopped=0
stop_car() {
  [ $stopped = 1 ] && return; stopped=1
  [ -n "${watcher:-}" ] && kill "$watcher" 2>/dev/null || true
  echo; echo "== stopping the race"
  in_ros "$kill_race; for i in \$(seq 20); do pgrep -f '$race_nodes' >/dev/null || exit 0; sleep 0.5; done; \
    for p in \$(pgrep -f '$race_nodes'); do pkill -KILL -P \$p; kill -KILL \$p; done" || true
  for _ in 1 2 3 4 5; do
    v=$(speed); [ "$v" != "?" ] && python3 -c "import sys; sys.exit(0 if $v < 0.05 else 1)" && break
    sleep 1
  done
  echo "== car speed ${v:-?} m/s"
}
trap stop_car EXIT
trap 'exit 130' INT TERM

watch_laps() {   # stop after $laps laps: lap_count rising (a sim reset drops it back, laps so far are kept)
  local done=0 last=
  sleep 5
  while [ $done -lt "$laps" ]; do
    out=$(in_ros "timeout 5 ros2 topic echo --once --field data /autodrive/roboracer_1/lap_count 2>/dev/null; \
      timeout 5 ros2 topic echo --once --field data /autodrive/roboracer_1/last_lap_time 2>/dev/null" | grep -v '^---' || true)
    n=$(sed -n 1p <<<"$out"); t=$(sed -n 2p <<<"$out")
    [[ "$n" =~ ^[0-9]+$ ]] || { sleep 1; continue; }
    if [ -z "$last" ] || [ "$n" -lt "$last" ]; then last=$n
    elif [ "$n" -gt "$last" ]; then done=$((done + n - last)); last=$n; printf '\n== lap %d/%d done (%.2f s)\n' $done "$laps" "$t"
    fi
    sleep 1
  done
  echo "== $laps laps done"
  in_ros "$kill_race" || true
}

speed=0.5; args=()
for a in "$@"; do
  case "$a" in speed:=*|speed_scale:=*) speed=${a#*:=} ;; *) args+=("$a") ;; esac
done
echo "== race $track${laps:+, $laps laps}: speed:=$speed localizer:=mcl avoid:=true ${args[*]-}"
watcher=; [ -n "$laps" ] && { watch_laps & watcher=$!; }
tty=(); [ -t 0 ] || tty=(-T)
"${compose[@]}" exec ${tty[@]+"${tty[@]}"} ros bash -c 'exec scripts/race.sh "$@"' race "$track" \
  speed_scale:="$speed" localizer:=mcl avoid:=true ${args[@]+"${args[@]}"} || true
