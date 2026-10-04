#!/usr/bin/env bash
# ForzaETH race stack on the AutoDRIVE simulator. Docs: docs/forzaeth_autodrive.md
#
#   ./forzaeth.sh up                    start the container (first run: builds image + workspace)
#   ./forzaeth.sh bridge                start the AutoDRIVE bridge (then click Connect in the sim)
#   ./forzaeth.sh map [name]            autopilot one lap, save the map   (default autodrive_roboracer)
#   ./forzaeth.sh plan [name]           ForzaETH global planner on that map + track.csv export
#   ./forzaeth.sh line <list|import|use|export> ...   raceline library (scripts/raceline_tool.py)
#   ./forzaeth.sh race --line forza_iqp --speed 0.5 --laps 10 [--experiment NAME]
#                                       one logged run: a standing lap + N timed laps, saved as
#                                       data/[NAME/]<map>_<line>_x<speed>_<YYYYmmdd_HHMMSS>/ + a snapshot
#   ./forzaeth.sh compare [dir] [--line X]   table of logged runs (default: all of data/ but archive/)
#   ./forzaeth.sh reset                 put the car back on the start line (API reset)
#   ./forzaeth.sh stop                  stop base/race/mapping (the bridge keeps running)
#   ./forzaeth.sh logs <bridge|base|race|mapping>    ./forzaeth.sh shell    ./forzaeth.sh down
#
# Start order: ./autodrive.sh sim, ./forzaeth.sh up, ./forzaeth.sh bridge, then connect the
# sim (127.0.0.1:4567) and switch it to autonomous. Leave the bridge running: restarting it
# drops the sim's connection.
set -euo pipefail
root="$(cd "$(dirname "$0")" && pwd)"
compose=(docker compose -f "$root/docker/forzaeth.compose.yml")
ctr=forzaeth_autodrive
MAP_DEFAULT=autodrive_roboracer
export USER="${USER:-$(id -un)}"
# Host layout (container paths are in docker/forzaeth.compose.yml)
stack="$root/planners/forzaeth/race_stack"           # ForzaETH clone   -> ~/ws/src/race_stack
pkg="$root/planners/forzaeth/autodrive_forzaeth"     # our package      -> ~/ws/src/autodrive_forzaeth
cache="$root/planners/forzaeth/build_cache/humble"   # colcon cache     -> ~/ws/{build,install,log}
racelines="$root/racelines"                 # raceline library -> ~/ws/racelines/<map>/<map>_<line>.json
data="$root/data"                           # run data         -> ~/ws/runs

# Run a command in the container with the workspace sourced.
in_ctr() { docker exec "$@"; }
ros() { in_ctr "$ctr" bash -c "source ~/ws/install/setup.bash && $1"; }
ros_it() {   # foreground; -t only when we have a terminal
  local t=(-i); [ -t 0 ] && t=(-it)
  in_ctr "${t[@]}" "$ctr" bash -c "source ~/ws/install/setup.bash && $1"
}
ros_bg() { in_ctr -d "$ctr" bash -c "source ~/ws/install/setup.bash && $1"; }

running() { [ "$(docker inspect -f '{{.State.Running}}' "$ctr" 2>/dev/null)" = true ]; }
need_up() { running || { echo "Container not running. Run: $0 up" >&2; exit 1; }; }

# Kill processes in the container whose command line matches an ERE (never this shell).
kill_matching() {
  in_ctr "$ctr" bash -c "ps -eo pid,args | grep -E -- '$1' | grep -v -e 'grep' -e 'ps -eo' | awk '{print \$1}' | xargs -r kill -INT 2>/dev/null; true"
}
STACK_RE='ros2 launch autodrive_forzaeth|lib/autodrive_forzaeth/(adapter|mapper|run_logger)|map_server|lifecycle_manager|frenet_odom|global_parameter|global_trajectory|sector_tuner|ot_interpolator|lap_analyser|foxglove_bridge|lib/controller/controller|lib/state_machine/state_machine'
stop_stack() {
  kill_matching "$STACK_RE"
  sleep 2
}

reset_car() {   # the bridge holds V1 Reset for a few frames, then releases it
  ros "ros2 topic pub -t 3 -w 1 /autodrive/reset_command std_msgs/msg/Bool '{data: true}'" >/dev/null
  sleep 1.5
}

colcon_build() {
  ros "cd ~/ws && colcon build --symlink-install --packages-skip f110_gym f1tenth_gym_ros \
       --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo $*" 2>&1 | grep -E 'Summary|Failed|failed|error:' || true
}

cmd="${1:-}"; shift || true
case "$cmd" in
  up)
    if ! docker image inspect race_stack:humble_arm >/dev/null 2>&1; then
      echo "Building ForzaETH base image (upstream compose, ~5 min)..."
      ( cd "$stack" &&
        printf 'ROS_DOMAIN_ID=48\nRACECAR_VERSION=SIM\nHOST_UID=%s\nHOST_GID=%s\n' "$(id -u)" "$(id -g)" > .env &&
        docker compose build arm )
    fi
    mkdir -p "$cache/"{build,install,log} "$racelines" "$data"
    "${compose[@]}" up -d --build
    # The TUM optimizer is pip-installed from the (mounted) source; lost if the container is recreated.
    in_ctr "$ctr" bash -c 'python3 -c "import global_racetrajectory_optimization" 2>/dev/null ||
      sudo pip3 install -q -e ~/ws/src/race_stack/planner/global_planner/global_planner/global_racetrajectory_optimization/'
    if ! in_ctr "$ctr" test -f /home/"$USER"/ws/install/setup.bash; then
      echo "First start: building the workspace..."
      in_ctr "$ctr" bash -c 'sudo chown -R $USER:$USER ~/ws/build ~/ws/install ~/ws/log'
      in_ctr "$ctr" bash -c "source /opt/ros/humble/setup.bash && cd ~/ws && colcon build --symlink-install \
        --packages-skip f110_gym f1tenth_gym_ros --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo" | tail -3
    fi
    echo "Up. Next: ./autodrive.sh sim (if not open), then $0 bridge" ;;

  build)
    need_up; colcon_build "$@" ;;

  bridge)
    need_up
    if in_ctr "$ctr" pgrep -f lib/autodrive_forzaeth/fast_bridge >/dev/null; then
      echo "Bridge already running (leave it: restarting it forces a reconnect in the sim)."
    else
      ros_bg "ros2 run autodrive_forzaeth fast_bridge > /tmp/bridge.log 2>&1"
      echo "Bridge started on :4567. In the sim: click Connect, set Driving Mode = Autonomous."
    fi
    echo "Waiting for sensor data..."
    for _ in $(seq 1 60); do
      if ros "timeout 3 ros2 topic echo --once /autodrive/f1tenth_1/ips" >/dev/null 2>&1; then
        echo "Sim connected and streaming."; exit 0
      fi
      sleep 2
    done
    echo "No data after 2 min. Is the sim open, connected, and on 127.0.0.1:4567?" >&2; exit 1 ;;

  map)
    need_up; map="${1:-$MAP_DEFAULT}"; speed="${2:-1.5}"
    stop_stack
    reset_car
    echo "Mapping '$map': the autopilot drives one slow lap from the spawn (which becomes the"
    echo "start/finish line), then saves. Ctrl-C saves early."
    ros_it "ros2 launch autodrive_forzaeth mapping_launch.xml map_name:=$map autopilot_speed:=$speed" || true
    stop_stack ;;

  plan)
    need_up; map="${1:-$MAP_DEFAULT}"
    mapdir="$stack/stack_master/maps/$map"
    [ -f "$mapdir/$map.png" ] || { echo "No map at $mapdir. Run: $0 map $map" >&2; exit 1; }
    echo "Running ForzaETH's global planner on '$map' (about a minute)..."
    before=$(stat -f %m "$mapdir/global_waypoints.json" 2>/dev/null || echo 0)
    # The planner node finishes but does not exit cleanly, so wait for its output file.
    ros_bg "cd ~ && ros2 run global_planner global_planner --ros-args -r __node:=global_planner \
      --params-file ~/ws/install/stack_master/share/stack_master/config/global_planner/global_planner_params.yaml \
      -p map_name:=$map > /tmp/plan.log 2>&1"
    seen=0
    for _ in $(seq 1 300); do
      now=$(stat -f %m "$mapdir/global_waypoints.json" 2>/dev/null || echo 0)
      [ "$now" != "$before" ] && break
      if in_ctr "$ctr" pgrep -f lib/global_planner/global_planner >/dev/null; then seen=1
      elif [ $seen = 1 ]; then break   # it died without writing output
      fi
      sleep 2
    done
    sleep 2; kill_matching 'lib/global_planner/global_planner'
    now=$(stat -f %m "$mapdir/global_waypoints.json" 2>/dev/null || echo 0)
    if [ "$now" = "$before" ]; then
      echo "Planner failed:" >&2; in_ctr "$ctr" tail -20 /tmp/plan.log >&2; exit 1
    fi
    n=$(python3 -c "import json;print(len(json.load(open('$mapdir/global_waypoints.json'))['global_traj_wpnts_iqp']['wpnts'])-1)")
    [ -f "$mapdir/speed_scaling.yaml" ] || printf 'sector_tuner:\n  ros__parameters:\n    global_limit: 0.3\n    n_sectors: 1\n    Sector0:\n      start: 0\n      end: %s\n      scaling: 0.3\n      only_FTG: false\n      no_FTG: false\n' "$n" > "$mapdir/speed_scaling.yaml"
    [ -f "$mapdir/ot_sectors.yaml" ] || printf 'ot_interpolator:\n  ros__parameters:\n    n_sectors: 1\n    yeet_factor: 1.25\n    spline_len: 30\n    ot_sector_begin: 0.5\n    Overtaking_sector0:\n      start: 0\n      end: %s\n      ot_flag: false\n' "$n" > "$mapdir/ot_sectors.yaml"
    python3 -c "import json;print(json.load(open('$mapdir/global_waypoints.json'))['map_info_str']['data'])"
    # A fresh plan replaces the library's ForzaETH lines.
    rm -f "$racelines/$map/${map}_forza_iqp.json" "$racelines/$map/${map}_forza_sp.json"
    ros "python3 ~/ws/src/autodrive_forzaeth/scripts/raceline_tool.py export $map"
    colcon_build --packages-select stack_master ;;

  line)
    need_up
    # import <map> <csv> <name>: the source CSV is kept in the library as
    # racelines/<map>/<map>_<name>.csv (the race snapshot copies it from there).
    if [ "${1:-}" = import ] && [ $# -ge 4 ] && [ -f "$3" ]; then
      map="$2"; name="${4#"${map}_"}"
      abs="$(cd "$(dirname "$3")" && pwd)/$(basename "$3")"
      dest="$racelines/$map/${map}_${name}.csv"
      mkdir -p "$racelines/$map"
      [ "$abs" = "$dest" ] || cp "$abs" "$dest"
      set -- import "$map" "$dest" "$name" "${@:5}"
    fi
    # Other CSV paths on the Mac are translated to the container's mounts.
    args=()
    for a in "$@"; do
      if [ -f "$a" ]; then
        abs="$(cd "$(dirname "$a")" && pwd)/$(basename "$a")"
        case "$abs" in
          "$racelines/"*) a="/home/$USER/ws/racelines/${abs#"$racelines/"}" ;;
          "$pkg/"*) a="/home/$USER/ws/src/autodrive_forzaeth/${abs#"$pkg/"}" ;;
          *) cp "$abs" "$racelines/"; a="/home/$USER/ws/racelines/$(basename "$abs")" ;;
        esac
      fi
      args+=("$a")
    done
    ros "python3 ~/ws/src/autodrive_forzaeth/scripts/raceline_tool.py ${args[*]}" ;;

  race)
    need_up
    map=$MAP_DEFAULT; line=""; speed=""; laps=5; ctrl=MAP; label=""; experiment=""
    while [ $# -gt 0 ]; do
      case "$1" in
        --map) map="$2"; shift 2 ;;
        --line) line="$2"; shift 2 ;;
        --speed) speed="$2"; shift 2 ;;
        --laps) laps="$2"; shift 2 ;;
        --ctrl) ctrl="$2"; shift 2 ;;
        --label) label="$2"; shift 2 ;;
        --experiment) experiment="$2"; shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
      esac
    done
    mapdir="$stack/stack_master/maps/$map"
    [ -n "$line" ] && ros "python3 ~/ws/src/autodrive_forzaeth/scripts/raceline_tool.py use $map $line"
    line="$(ros "python3 ~/ws/src/autodrive_forzaeth/scripts/raceline_tool.py active $map")"
    if [ -n "$speed" ]; then
      sed -i '' -E "s/(global_limit|scaling): .*/\1: $speed/" "$mapdir/speed_scaling.yaml"
    fi
    speed="$(sed -nE 's/.*global_limit: (.*)/\1/p' "$mapdir/speed_scaling.yaml")"
    label="${label:-${map}_${line}_x${speed}}"   # run_logger saves to <label>_<YYYYmmdd_HHMMSS>/
    runs_host="$data${experiment:+/$experiment}"
    runs_ctr="/home/$USER/ws/runs${experiment:+/$experiment}"
    mkdir -p "$runs_host"
    before_runs="$(ls "$runs_host")"
    stop_stack
    echo "Racing '$line' at ${speed}x speed for $laps laps (log label $label)."
    reset_car
    ros_bg "ros2 launch autodrive_forzaeth base_launch.xml map_name:=$map > /tmp/base.log 2>&1"
    for _ in $(seq 1 30); do
      ros "ros2 topic list" 2>/dev/null | grep -q '^/car_state/frenet/odom$' && break; sleep 1
    done
    sleep 3
    ros_it "ros2 launch autodrive_forzaeth race_launch.xml map_name:=$map raceline:=$label \
      max_laps:=$laps ctrl_algo:=$ctrl runs_dir:=$runs_ctr 2>&1 | tee /tmp/race.log" || true
    stop_stack
    run_dir="$(comm -13 <(echo "$before_runs") <(ls "$runs_host") | grep -E '_[0-9]{8}_[0-9]{6}$' | tail -1)"
    if [ -n "$run_dir" ]; then
      # Snapshot exactly what was driven, so the run can be reproduced and audited later.
      d="$runs_host/$run_dir"
      lib="$racelines/$map"
      cp "$mapdir/global_waypoints.json" "$d/global_waypoints.json"
      cp "$mapdir/speed_scaling.yaml" "$d/speed_scaling.yaml"
      [ -f "$lib/${map}_${line}.csv" ] && cp "$lib/${map}_${line}.csv" "$d/raceline_source.csv"
      python3 - "$d" "$line" "$speed" "$laps" "$map" "$ctrl" "$experiment" "$stack" "$label" <<'PY'
import json, os, subprocess, sys, datetime
d, line, speed, laps, mp, ctrl, exp, stack, label = sys.argv[1:]
def git(*a):
    try:
        return subprocess.run(['git', '-C', stack, *a],
                              capture_output=True, text=True).stdout.strip()
    except Exception:
        return None
info = json.load(open(f'{d}/global_waypoints.json'))['map_info_str']['data']
meta = {'experiment': exp or None, 'raceline': label, 'track': mp, 'line': line,
        'speed_scaling': float(speed), 'timed_laps_requested': int(laps),
        'map': mp, 'controller': ctrl, 'lu_table': 'SIM_linear', 'recorded': datetime.datetime.now().isoformat(timespec='seconds'),
        'raceline_info': info,
        'simulator': 'AutoDRIVE RoboRacer 2026-iros (practice, macOS)',
        'forzaeth': {'branch': 'ros2-humble', 'commit': git('rev-parse', 'HEAD'),
                     'local_patches': git('diff', '--stat').splitlines()},
        'files': {'global_waypoints.json': 'exact waypoints + speed profile driven (before speed scaling)',
                  'speed_scaling.yaml': 'ForzaETH sector speed scaling used',
                  'raceline_source.csv': 'raceline as generated (TUM traj_race_cl format), if imported'}}
json.dump(meta, open(f'{d}/metadata.json', 'w'), indent=2)
sp = f'{d}/summary.json'   # tag the summary too, so compare can group by track/line
if os.path.isfile(sp):
    s = json.load(open(sp))
    s.update(track=mp, line=line, speed_scaling=float(speed))
    json.dump(s, open(sp, 'w'), indent=2)
PY
      echo "Saved run: $d"
    fi
    python3 "$pkg/scripts/compare_runs.py" "$runs_host" --last 1 ;;

  compare)
    python3 "$pkg/scripts/compare_runs.py" "$@" ;;

  reset)
    need_up; reset_car; echo "Car reset to the start line." ;;

  stop)
    need_up; stop_stack; echo "Stopped (bridge still running)." ;;

  logs)
    need_up; in_ctr "$ctr" tail -n 50 -f "/tmp/${1:-race}.log" ;;

  shell)
    need_up; in_ctr -it "$ctr" bash ;;

  down)
    "${compose[@]}" down ;;

  *)
    sed -n '2,17p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
