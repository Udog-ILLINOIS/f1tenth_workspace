#!/usr/bin/env bash
# Launch the AutoDRIVE RoboRacer simulator (2026-iros practice build + TrackSelect plugin), from anywhere.
#   ./autodrive.sh fetch        download the multi-track app from our AutoDRIVE fork's release
#                               (built by autodrive/track_builder, branch f1tenth_multitrack)
#   ./autodrive.sh sim          1280x720 window, "Very Low" quality (higher sensor rate), Porto
#   ./autodrive.sh sim --full   the app's own resolution/quality settings
#   ./autodrive.sh sim --track NAME   started with -track NAME. Built in: porto (default), berlin,
#       srl2024cdc, srl2024iros, srl2025icra, srl2025cdctf; custom: iros2026
#   ./autodrive.sh sim --no-connect   don't connect to the bridge (127.0.0.1:4567) by itself
#   ./autodrive.sh sim --cars N   N cars (MultiCar.cs). Car k's bridge is on port 4566+k (car 1: 4567),
#       one driver container each (./getup.sh --driver k); race them with ./race_cars.sh. Options:
#     --ghost 0|1        1: the other cars pass through, look translucent blue and the lidars do not see
#                        them (= --contact 0 --lidar-cars 0); 0: solid, colliding, seen. Also the Menu's
#                        Ghost row (shown with 2+ cars) and ctl ghost on|off
#     --contact 0|1      only the collisions (and the look) of the other cars; ctl contact on|off
#     --lidar-cars 0|1   only whether the lidars see the other cars; ctl lidarcars on|off
#     --carlink 0|1      1 (default): car 1 talks to its bridge through the plugin's CarLink, whose frames
#                        carry V1 Sim Time (the bridge stamps with it); 0: the sim's stock Socket (one car)
#     --grid "b,l;b,l"   start of car 2, 3.. in m behind / left of car 1 (default 1.5 m back per car)
#     ctl cars | place CAR X Y [YAW] | resetcar CAR | hold [on|off] | start | park|unpark CAR | ghostlook ...
#   ./autodrive.sh ctl CMD      send CMD to the sim's control port (127.0.0.1:4580), e.g. status, tracks,
#       track NAME, connect, disconnect, reconnect, kick, click BUTTON, screenshot [PATH]
#       (SimControl in autodrive/track_builder/track_select; the sim also has a Map button)
# The ROS 2 side is one of two stacks, each with its own bridge on port 4567 (one at a time):
# the club stack (planners/sigrobotics, docker compose) or ForzaETH (./forzaeth.sh); see docs/autodrive.md.
# The devkit (autodrive/devkit, branch AutoDRIVE-Devkit of our AutoDRIVE fork) is kept for
# reference; both stacks use their own fast_bridge instead.
set -euo pipefail
root="$(cd "$(dirname "$0")" && pwd)"
dir="$root/autodrive/simulator_multitrack"
app="$dir/AutoDRIVE Simulator.app"
release=https://github.com/Udog-ILLINOIS/AutoDRIVE/releases/download/f1tenth_multitrack_v3/autodrive_simulator_multitrack_macos.zip

case "${1:-}" in
  fetch)
    [ -d "$app" ] && { echo "Already installed: $app" >&2; exit 0; }
    mkdir -p "$dir"; tmp="$(mktemp -d)"
    curl -fL -o "$tmp/sim.zip" "$release"
    ditto -x -k "$tmp/sim.zip" "$dir"; rm -rf "$tmp"
    xattr -cr "$app"; echo "Installed $app" ;;
  sim)
    shift; full=0; track=(); connect=(-autoconnect); multi=()
    while [ $# -gt 0 ]; do
      case "$1" in
        --full) full=1; shift ;;
        --track) track=(-track "$2"); shift 2 ;;
        --no-connect) connect=(); shift ;;
        --cars) multi+=(-cars "$2"); shift 2 ;;
        --contact) multi+=(-contact "$2"); shift 2 ;;
        --grid) multi+=(-grid "$2"); shift 2 ;;
        --lidar-cars) multi+=(-lidar-cars "$2"); shift 2 ;;
        --ghost) multi+=(-ghost "$2"); shift 2 ;;
        --carlink) multi+=(-carlink "$2"); shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
      esac
    done
    [ -d "$app" ] || { echo "Simulator not found at $app (run: $0 fetch)" >&2; exit 1; }
    # -n: open a new instance even if another copy of the sim is running
    # The sim sends one sensor frame per rendered frame, so a small low-quality window
    # gives ForzaETH a faster control loop.
    if [ $full = 1 ]; then exec open -n -a "$app" --args ${track[@]+"${track[@]}"} ${connect[@]+"${connect[@]}"} ${multi[@]+"${multi[@]}"}; fi
    exec open -n -a "$app" --args -screen-fullscreen 0 -screen-width 1280 -screen-height 720 \
      -screen-quality "Very Low" ${track[@]+"${track[@]}"} ${connect[@]+"${connect[@]}"} ${multi[@]+"${multi[@]}"} ;;
  ctl)
    shift; [ $# -gt 0 ] || { echo "usage: $0 ctl status|tracks|track NAME|connect|disconnect|reconnect|kick|click BUTTON|screenshot [PATH]" >&2; exit 2; }
    printf '%s\n' "$*" | nc -w 15 127.0.0.1 "${AUTODRIVE_CONTROL_PORT:-4580}" \
      || { echo "no answer on the sim's control port (is the sim running?)" >&2; exit 1; } ;;
  *)
    echo "usage: $0 fetch | sim [--full] [--track NAME] [--no-connect] [--cars N] [--contact 0|1] [--grid G] [--lidar-cars 0|1] [--ghost 0|1] [--carlink 0|1] | ctl CMD" >&2; exit 2 ;;
esac
