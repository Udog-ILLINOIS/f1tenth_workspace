#!/usr/bin/env bash
# Launch the AutoDRIVE RoboRacer simulator (2026-iros practice build), from anywhere.
#   ./autodrive.sh sim          1280x720 window, "Very Low" quality (higher sensor rate)
#   ./autodrive.sh sim --full   the app's own resolution/quality settings
#   ./autodrive.sh sim --track NAME   the multi-track copy (autodrive/simulator_multitrack/, built by
#       autodrive/track_builder/build_track.py) started with -track NAME. Built in: porto, berlin,
#       srl2024cdc, srl2024iros, srl2025icra, srl2025cdctf; custom: iros2026
# The ROS 2 side lives in the ForzaETH container: see ./forzaeth.sh and docs/forzaeth_autodrive.md.
# The devkit's own ROS 2 bridge (autodrive/roboracer_sim/autodrive_devkit, package autodrive_roboracer) is kept for
# reference; forzaeth.sh uses planners/forzaeth/autodrive_forzaeth's fast_bridge instead.
set -euo pipefail
root="$(cd "$(dirname "$0")" && pwd)"
app="$root/autodrive/simulator/AutoDRIVE Simulator.app"

case "${1:-}" in
  sim)
    shift; full=0; track=()
    while [ $# -gt 0 ]; do
      case "$1" in
        --full) full=1; shift ;;
        --track) app="$root/autodrive/simulator_multitrack/AutoDRIVE Simulator.app"; track=(-track "$2"); shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
      esac
    done
    [ -d "$app" ] || { echo "Simulator not found at $app" >&2; exit 1; }
    # -n: the stock and custom-track apps share a bundle id, so open a new instance of this one
    # The sim sends one sensor frame per rendered frame, so a small low-quality window
    # gives ForzaETH a faster control loop.
    if [ $full = 1 ]; then exec open -n -a "$app" --args ${track[@]+"${track[@]}"}; fi
    exec open -n -a "$app" --args -screen-fullscreen 0 -screen-width 1280 -screen-height 720 \
      -screen-quality "Very Low" ${track[@]+"${track[@]}"} ;;
  *)
    echo "usage: $0 sim [--full] [--track NAME]" >&2; exit 2 ;;
esac
