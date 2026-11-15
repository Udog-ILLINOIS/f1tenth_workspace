#!/usr/bin/env bash
# Race the sim's current track: club stack race.sh with MCL localization and obstacle avoidance.
#   ./go.sh [TRACK] [LAPS] [race.sh args...]   TRACK: the sim's current track by default
#   ./go.sh                    race until Ctrl-C
#   ./go.sh 5                  stop the car after 5 laps
#   ./go.sh berlin 3 speed:=0.7                defaults: speed:=0.5 localizer:=mcl avoid:=true
#   speed:=S is race.sh's speed_scale:=S (drive speed = raceline vx * S)
#   ./go.sh --max:=true        fastest lap: TRACK_grip raceline (built from TRACK_grip/racecar_grip.ini),
#                              speed:=1.1, avoid:=false, adapter max_acceleration 4.5 (porto: 5.70 s, ~1 crash/min)
#   ./go.sh --driver 2 [TRACK] ...   drive car 2 of a --cars sim with driver 2's container (port 4568,
#                              ROS domain 2); the same flag on getup.sh starts it. Driver 1 is the default.
# Needs ./getup.sh first (container, bridge, connected sim). Ctrl-C (or the last lap) stops the race
# nodes; the bridge then zeroes the car's commands (fast_bridge cmd_timeout) and waits until it stands.
# The stack runs on the sim's clock: race.sh gets use_sim_time:=true (unless given use_sim_time:=...) and the
# restarted adapter -p use_sim_time:=true; getup.sh's bridge publishes /clock (fast_bridge use_sim_time:=true).
set -euo pipefail
root="$(cd "$(dirname "$0")" && pwd)"
driver=1
if [ "${1:-}" = "--driver" ]; then driver=$2; shift 2; fi
source "$root/autodrive/driver_env.sh" "$driver"   # compose=(...) for this driver's container, car_connected
in_ros() { "${compose[@]}" exec -T ros bash -c "source /opt/ros/humble/setup.bash; $1"; }

car_connected || { echo "car $driver not connected to its bridge: run ./getup.sh${driver_flag} first" >&2; exit 1; }
# (race_cars.sh holds the cars on purpose until every stack is ready: RACE_CARS=1)
if [ -z "${RACE_CARS:-}" ] && "$root/autodrive.sh" ctl cars 2>/dev/null | python3 -c "import json,sys; d=json.load(sys.stdin); sys.exit(0 if d.get('held') or any(c.get('parked') and c['car']==$driver for c in d['cars']) else 1)"; then
  echo "the sim is holding the cars or has car $driver parked (race_cars.sh): ./autodrive.sh ctl hold off; ./autodrive.sh ctl unpark $driver" >&2
  exit 1
fi

max=false; rest=()
for a in "$@"; do
  case "$a" in --max|--max:=true|max:=true) max=true ;; --max:=false|max:=false) ;; *) rest+=("$a") ;; esac
done
set -- ${rest[@]+"${rest[@]}"}

track=; laps=
if [ $# -gt 0 ] && [[ "$1" != *:=* && ! "$1" =~ ^[0-9]+$ ]]; then track=$1; shift; fi
if [ $# -gt 0 ] && [[ "$1" =~ ^[0-9]+$ ]]; then laps=$1; shift; fi
sim_track=$(python3 "$root/autodrive/map_feed.py" --current)
[ -n "$track" ] || track=$sim_track
tracks="$root/planners/sigrobotics/ws/tracks"
if [ $max = true ] && [[ "$track" != *_grip ]]; then
  base=$(tr '[:upper:]' '[:lower:]' <<<"$track")   # sim names are capitalized (Porto)
  [ -f "$tracks/${base}_grip/raceline.csv" ] || { echo "--max: no '${base}_grip' raceline. Build one with" \
    "build_raceline ${base}_grip --config <higher-grip racecar.ini> (see tracks/porto_grip/racecar_grip.ini)" >&2; exit 1; }
  track=${base}_grip
fi
if [ ! -f "$tracks/$track/raceline.csv" ]; then
  echo "no mapped track '$track' (tracks with a raceline: $(cd "$tracks" && ls */raceline.csv | sed 's|/raceline.csv||' | tr '\n' ' '))" >&2
  trap - EXIT; exit 1
fi
if [ "$track" != "$sim_track" ] && [ "$driver" != 1 ]; then
  base=${track%%_*}; [ "$base" = "$sim_track" ] || { echo "the sim is on $sim_track: only driver 1 switches tracks" >&2; exit 1; }
elif [ "$track" != "$sim_track" ]; then   # race.sh drives this track's raceline: put the sim on it too
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

lap_state() {   # "<lap count> <last lap s|?>" of this driver's car, both from ONE sim snapshot (ctl cars)
  # (the ROS lap_count and last_lap_time topics are separate messages: read one after the other they can
  # belong to different laps, and under load an echo can print "A message was lost!!!" instead of a value)
  local s
  s=$("$root/autodrive.sh" ctl cars 2>/dev/null | python3 -c "
import json, sys
c = [c for c in json.load(sys.stdin)['cars'] if c['car'] == $driver][0]
print(int(c['laps']), '?' if c.get('last_lap') is None else '%.2f' % c['last_lap'])" 2>/dev/null) && { echo "$s"; return; }
  # older plugin (no ctl cars): the topics, numbers only
  s=$(in_ros "timeout 5 ros2 topic echo --once --field data /autodrive/roboracer_1/lap_count 2>/dev/null" \
    | grep -m1 -xE '[0-9]+' || true)
  [ -n "$s" ] && echo "$s ?"
}
watch_laps() {   # stop after $laps laps: lap count rising (a sim reset drops it back, laps so far are kept)
  local done=0 last= n t
  sleep 5
  while [ $done -lt "$laps" ]; do
    read -r n t <<<"$(lap_state || true)" || true
    if ! [[ "${n:-}" =~ ^[0-9]+$ ]]; then sleep 1; continue; fi
    if [ -z "$last" ] || [ "$n" -lt "$last" ]; then last=$n
    elif [ "$n" -gt "$last" ]; then
      done=$((done + n - last)); last=$n
      echo; echo "== lap $done/$laps done (${t:-?} s)"
    fi
    sleep 1
  done
  echo "== $laps laps done"
  in_ros "$kill_race" || true
}

adapter_accel() {   # max_acceleration is read once at start: restart the bridge's adapter node to change it
  local cur
  for _ in 1 2 3; do   # discovery can be slow right after getup.sh
    cur=$(in_ros "ros2 param get /adapter max_acceleration 2>/dev/null" | grep -o '[0-9.]*$' || true)
    [ -n "$cur" ] && break; sleep 1
  done
  [ "$cur" = "$1" ] && return
  # unknown: leave the bridge launch's own adapter alone (a second one would drive the same car)
  [ -n "$cur" ] || { echo "== adapter max_acceleration unknown: not restarting it (wanted $1)" >&2; return; }
  echo "== adapter max_acceleration ${cur:-?} -> $1 (restarting the adapter node)"
  in_ros "pkill -INT -f '^/usr/bin/python3 /root/ws/install/autodrive_bridge/lib/autodrive_bridge/adapter '; sleep 1.5; \
    pkill -KILL -f '^/usr/bin/python3 /root/ws/install/autodrive_bridge/lib/autodrive_bridge/adapter ' || true; \
    sed -E 's/^( *max_acceleration:) *[0-9.]+/\1 $1/' install/autodrive_bridge/share/autodrive_bridge/config/adapter.yaml \
      > /tmp/adapter_go.yaml" || true
  "${compose[@]}" exec -d ros bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; \
    exec ros2 run autodrive_bridge adapter --ros-args -r __node:=adapter \
      --params-file install/control/share/control/config/vehicle.yaml --params-file /tmp/adapter_go.yaml \
      -p use_sim_time:=true \
      > /tmp/adapter_go.log 2>&1"
  sleep 3
}

speed=0.5; avoid=true; args=()
[ $max = true ] && { speed=1.1; avoid=false; }
for a in "$@"; do
  case "$a" in speed:=*|speed_scale:=*) speed=${a#*:=} ;; avoid:=*) avoid=${a#*:=} ;; *) args+=("$a") ;; esac
done
# --max drives porto_grip at 1.1: its raceline alone asks ~12 m/s^2, past pure pursuit's default grip
# limit (max_lateral_accel 9.5, set for the stock raceline): off unless given
if [ $max = true ] && ! printf '%s\n' ${args[@]+"${args[@]}"} | grep -q '^max_lateral_accel:='; then
  args+=(max_lateral_accel:=0)
fi
# the sim's clock (fast_bridge publishes /clock, getup.sh), unless given
sim_time=true; for a in ${args[@]+"${args[@]}"}; do [[ "$a" == use_sim_time:=* ]] && sim_time=; done
[ -n "$sim_time" ] && args+=(use_sim_time:=true)
if [ $max = true ]; then
  adapter_accel 4.5
  echo "== MAX: $(grep -m1 '^# raceline' "$tracks/$track/raceline.csv" | grep -o '"lap_s": [0-9.]*') plan," \
    "speed:=$speed, avoid:=$avoid, accel 4.5 m/s^2: crashes are expected"
else
  adapter_accel 2.5
fi
echo "== race $track${laps:+, $laps laps}: speed:=$speed localizer:=mcl avoid:=$avoid ${args[*]-}"
watcher=; [ -n "$laps" ] && { watch_laps & watcher=$!; }
tty=(); [ -t 0 ] || tty=(-T)
"${compose[@]}" exec ${tty[@]+"${tty[@]}"} ros bash -c 'exec scripts/race.sh "$@"' race "$track" \
  speed_scale:="$speed" localizer:=mcl avoid:="$avoid" ${args[@]+"${args[@]}"} || true
