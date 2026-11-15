#!/usr/bin/env bash
# Race every car of a multi-car sim at once, each driven by its own driver's club stack (go.sh --driver N),
# all released in the same sim frame.
#   ./race_cars.sh [TRACK] [LAPS] [race.sh args...]     e.g. ./race_cars.sh porto 5
#   defaults: the sim's track, 5 laps, speed:=1.0 (go.sh's own default is 0.5), avoid:=false (no obstacle
#   avoidance: with two stacks its phantom stops made the cars stop-start; avoid:=true turns it back on);
#   --max passes go.sh --max
#   --only=K: race car K alone (solo run: only driver K's stack drives, the other cars stand still with zero
#   throttle), e.g. to record each car without the others' load on the host; ghost_replay.py overlays runs
#   ./race_cars.sh --solo [TRACK] [LAPS] ...: --only=1, then --only=2, ... back to back, then the replay
#   start: with Ghost on (no contact, lidars not seeing the cars), every car starts exactly on car 1's spot
#   (--grid: the grid slots instead); otherwise always the grid slots
# Needs: the sim (one car, or --cars N: ./autodrive.sh sim --cars 2 [--ghost 1]) and every driver up
# (./getup.sh, ./getup.sh --driver 2, ...). The cars are held (zero throttle, MultiCar.cs) and reset to
# their grid slots, every driver's race starts, and once every stack has asked to drive for 2 s straight
# the sim releases them all in one frame with their lap timers restarted.
# Logs: data/head_to_head/<track>_x<speed>_<stamp>/: driver<N>.log, laps.csv (sim lap times), and
# truth_driver<N>.csv / truth_report.txt / truth_driver<N>.png: each stack's pose estimate vs the sim's ground
# truth, wheel slip and raceline offset, per crash (autodrive/truth_recorder.py, truth_analysis.py).
# Ctrl-C stops every race.
set -euo pipefail
root="$(cd "$(dirname "$0")" && pwd)"
ctl() { "$root/autodrive.sh" ctl "$@"; }
json() { python3 -c "import json,sys; d=json.load(sys.stdin); print($1)"; }

cars=$(ctl cars | json "len(d['cars'])")
[ "$cars" -ge 1 ] || { echo "the sim reports no car (ctl cars): is it up?" >&2; exit 1; }   # 1 car: a solo race with the same logging
for k in $(seq 1 "$cars"); do   # car_connected (autodrive/driver_env.sh) for each driver
  ( source "$root/autodrive/driver_env.sh" "$k"; car_connected ) \
    || { echo "car $k is not connected to its bridge: ./getup.sh$([ $k = 1 ] || echo " --driver $k")" >&2; exit 1; }
done

args=() track= laps=5 speed=1.0 max= grid=0 avoid=false only= solo=0
for a in "$@"; do
  case "$a" in
    --max|--max:=true|max:=true) max=--max ;;
    --grid) grid=1 ;;
    --only=*) only=${a#*=} ;;
    --solo) solo=1 ;;
    avoid:=*) avoid=${a#*:=} ;;
    speed:=*|speed_scale:=*) speed=${a#*:=} ;;
    *) if [ -z "$track" ] && [ ${#args[@]} = 0 ] && [[ "$a" != *:=* && ! "$a" =~ ^[0-9]+$ ]]; then track=$a
       elif [[ "$a" =~ ^[0-9]+$ ]]; then laps=$a
       else args+=("$a"); fi ;;
  esac
done
[ -n "$track" ] || track=$(python3 "$root/autodrive/map_feed.py" --current)
[ -n "$max" ] && speed=1.1
if [ $solo = 1 ]; then   # each car alone, one after the other, then all runs replayed together
  dirs=() pass=()
  for a in "$@"; do [ "$a" = --solo ] || pass+=("$a"); done
  for k in $(seq 1 "$cars"); do
    echo "######## solo run: car $k alone"
    "$0" ${pass[@]+"${pass[@]}"} --only=$k | tee /dev/stderr | grep '^== logs: ' > "/tmp/.solo_out_$$" || true
    d=$(sed 's/^== logs: //' "/tmp/.solo_out_$$"); rm -f "/tmp/.solo_out_$$"
    [ -n "$d" ] || { echo "car $k's solo run gave no logs" >&2; exit 1; }
    dirs+=("$d")
  done
  echo "######## replay: every solo run at once (autodrive/ghost_replay.py)"
  exec python3 "$root/autodrive/ghost_replay.py" "${dirs[@]}"
fi
active=$(seq 1 "$cars")
if [ -n "$only" ]; then
  [[ "$only" =~ ^[0-9]+$ ]] && [ "$only" -ge 1 ] && [ "$only" -le "$cars" ] || { echo "--only=K: K is 1..$cars" >&2; exit 1; }
  active=$only
fi
active_list=$(echo $active | tr ' ' ',')
final="$root/data/head_to_head/${track}_x${speed}${only:+_solo$only}_$(date +%Y%m%d_%H%M%S)"
# everything is written to a local scratch folder during the race and moved into data/ at the end: this repo
# sits in the iCloud-synced Desktop, and a race's constant small writes (5 lap polls/s, the truth recorders)
# kept iCloud's file sync (fileproviderd, opendirectoryd, automountd) at ~1 core, starving the sim
out=$(mktemp -d "${TMPDIR:-/tmp}/race_cars.XXXXXX")/$(basename "$final")
mkdir -p "$out"
save_out() {   # the race folder into data/ (once)
  [ -d "$out" ] && [ "$out" != "$final" ] || return 0
  mkdir -p "$(dirname "$final")" && mv "$out" "$final" && rmdir "$(dirname "$out")" 2>/dev/null; out=$final
}

echo "== holding $cars cars and putting them on the grid"
ctl hold >/dev/null
ctl grid >/dev/null   # every car's reset pose on its grid slot (an earlier ghost race may have stacked them)
for k in $(seq 1 "$cars"); do ctl resetcar "$k" >/dev/null; done
stacked=0
# stack the cars on car 1's spot only when they cannot touch AND the lidars cannot see them: a lidar that
# sees the other cars would start inside one and see a wall all around
if [ $grid = 0 ] && ctl cars | json "d['ghost'] and not d['lidar_sees_cars']" | grep -q True; then
  pose=$(ctl cars | json "' '.join(str(d['cars'][0][k]) for k in ('x','y','yaw'))")
  for k in $(seq 2 "$cars"); do ctl place "$k" $pose >/dev/null; done
  stacked=1
  echo "== ghosts: all $cars cars on car 1's start ($pose)"
fi
# lidars blind to the other cars while the stacks start and localize: a car parked 1.5 m ahead (Ghost off,
# grid start) made car 2's localizer place itself 11 m away and crash on lap 1 (2026-11-14). They see the
# cars again LIDAR_CARS_AFTER s after the start (restored on exit too)
lidar_cars_restore=
if [ "$cars" -gt 1 ] && ctl cars | json "d['lidar_sees_cars']" | grep -q True; then
  ctl lidarcars off >/dev/null; lidar_cars_restore=on
  echo "== lidars blind to the other cars until ${LIDAR_CARS_AFTER:-5} s after the start"
fi
sleep 2   # the bridges see the reset and settle before the stacks start

pids=() recs=()
# truth recorders (autodrive/truth_recorder.py, read-only): one per driver container, the stack's pose
# estimate, wheel odometry and commands next to the sim's ground truth -> truth_driver<k>.csv
rec_in() { ( source "$root/autodrive/driver_env.sh" "$1"; "${compose[@]}" exec -T ros bash -c "$2" ); }
start_recorders() {
  for k in $active; do
    rec_in "$k" "cat > /tmp/truth_recorder.py" < "$root/autodrive/truth_recorder.py"
    rec_in "$k" "source /opt/ros/humble/setup.bash; exec python3 -u /tmp/truth_recorder.py" \
      > "$out/truth_driver$k.csv" 2> "$out/truth_driver$k.err" < /dev/null &
    recs+=($!)
  done
}
stop_recorders() {
  [ ${#recs[@]} = 0 ] && return
  for k in $active; do rec_in "$k" "pkill -INT -f /tmp/truth_recorder.py" >/dev/null 2>&1 || true; done
  for p in "${recs[@]}"; do wait "$p" 2>/dev/null || true; done
  recs=()
}
stop_driver() { pkill -TERM -P "$1" 2>/dev/null || true; kill -TERM "$1" 2>/dev/null || true; }
stop() {
  trap - EXIT INT TERM
  # SIGTERM: a script's background jobs start with SIGINT ignored, which go.sh cannot trap again. Also to
  # go.sh's children: go.sh runs its trap (stopping the race nodes) only once its docker exec returns
  for p in ${pids[@]+"${pids[@]}"}; do stop_driver "$p"; done
  stop_recorders
  wait 2>/dev/null || true
  ctl hold off >/dev/null 2>&1 || true   # never leave the sim holding cars (ctl start is not needed after)
  [ -z "${lidar_cars_restore:-}" ] || ctl lidarcars on >/dev/null 2>&1 || true
  [ "${stacked:-0}" = 0 ] || ctl grid >/dev/null 2>&1 || true   # reset poses back on the grid slots
  save_out || true
}
trap stop EXIT
trap 'exit 130' INT TERM

start_recorders
for k in $active; do
  drv=(); [ "$k" = 1 ] || drv=(--driver "$k")
  speed_arg=(); [ -n "$max" ] || speed_arg=(speed:="$speed")
  RACE_CARS=1 "$root/go.sh" ${drv[@]+"${drv[@]}"} ${max:+$max} "$track" "$laps" ${speed_arg[@]+"${speed_arg[@]}"} \
    avoid:="$avoid" ${args[@]+"${args[@]}"} > "$out/driver$k.log" 2>&1 < /dev/null &
  pids[$k]=$!   # indexed by car
  echo "== driver $k starting (log: $out/driver$k.log)"
done

echo -n "== waiting for every stack to ask to drive"
ready=0
for _ in $(seq 240); do
  if ctl cars | json "all(c.get('wants_to_drive') for c in d['cars'] if c['car'] in [$active_list])" | grep -q True; then ready=1; break; fi
  for p in "${pids[@]}"; do kill -0 "$p" 2>/dev/null || { echo; echo "a driver exited before the start (see its log)" >&2; exit 1; }; done
  echo -n .; sleep 0.5
done
[ $ready = 1 ] || { echo; echo "not every stack asked to drive within 120 s (see the logs)" >&2; exit 1; }
echo; echo "== GO: $(ctl start | json "d['result']")"
if [ -n "$lidar_cars_restore" ]; then   # back on once the cars have localized and pulled apart
  ( sleep "${LIDAR_CARS_AFTER:-5}"; ctl lidarcars on >/dev/null 2>&1 && echo "== lidars see the other cars again" ) &
fi

# Lap times come from the sim: one `ctl cars` snapshot holds each car's lap count and last lap time
# together (sim time, crash laps included). Each car is parked the moment it has done its laps (a car left at
# speed would coast into the next wall) and its driver's go.sh is stopped right away (a driver never
# keeps a race waiting). laps.csv: car, lap, lap time, collisions so far.
echo "car,lap,time_s,collisions" > "$out/laps.csv"
stopped=()
while :; do
  ctl cars > "$out/.cars.json" 2>/dev/null || { sleep 0.2; continue; }
  python3 - "$out" "$laps" <<'E' | while read -r what k n t c; do
import json, os, sys
out, laps = sys.argv[1], int(sys.argv[2])
try:   # an error reply ({"ok":false,...}) or an empty one: skip this poll, never end the race
    cars = json.load(open(f"{out}/.cars.json"))["cars"]
except (ValueError, KeyError, TypeError):
    sys.exit(0)
f = f"{out}/.laps.json"
seen = json.load(open(f)) if os.path.exists(f) else {}
for c in cars:
    k = str(c["car"]); n = int(c["laps"]); last = seen.get(k, 0)
    if n > last:   # one poll is 0.2 s: two laps between polls cannot happen, but the count is what counts
        t = c.get("last_lap")
        for m in range(last + 1, n + 1):
            print("lap", k, m, "?" if t is None or m != n else f"{t:.2f}", c["collisions"])
        seen[k] = n
    if n >= laps and not c["parked"]:
        print("park", k, n, "-", c["collisions"])
json.dump(seen, open(f, "w"))
E
    case $what in
      lap) echo "== car $k lap $n/$laps: $t s (collisions $c)"; echo "$k,$n,$t,$c" >> "$out/laps.csv" ;;
      park) ctl park "$k" >/dev/null || true; echo "== car $k finished: parked" ;;
    esac
  done || true
  # stop the driver of every parked car that has done its laps
  for k in $(ctl cars 2>/dev/null | json "' '.join(str(c['car']) for c in d.get('cars', []) if c['parked'] and c['laps'] >= $laps)" 2>/dev/null || true); do
    [[ " ${stopped[*]-} " == *" $k "* ]] && continue
    stopped+=("$k"); stop_driver "${pids[$k]}"
  done
  alive=0; for p in "${pids[@]}"; do kill -0 "$p" 2>/dev/null && alive=1; done
  [ $alive = 1 ] || break
  sleep 0.2
done
for p in "${pids[@]}"; do wait "$p" || true; done
pids=()
ctl hold off >/dev/null   # finished cars stay parked; later runs (go.sh, tests) are not held
[ $stacked = 0 ] || ctl grid >/dev/null   # reset poses back on the grid slots (cars stay where they parked)
rm -f "$out/.cars.json" "$out/.laps.json"
stop_recorders
echo "== results ($track, $laps laps, speed $speed, avoid $avoid; sim lap times)"
python3 - "$out/laps.csv" $active <<'E'
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1])))
for k in map(int, sys.argv[2:]):
    r = [x for x in rows if x["car"] == str(k)]
    t = [float(x["time_s"]) for x in r if x["time_s"] != "?"]
    col = r[-1]["collisions"] if r else "0"
    print(f"car {k}: {len(r)} laps, collisions {col}" + (f", total {sum(t):.2f} s, best {min(t):.2f} s, laps "
          + " ".join(x["time_s"] for x in r) if t else ""))
E
echo "== measured vs ground truth (truth_report.txt, truth_driver<k>.png)"
"$root/autodrive/track_builder/venv/bin/python" "$root/autodrive/truth_analysis.py" "$out" "$track" \
  || echo "(truth analysis failed)"
save_out
echo "== logs: $out"
