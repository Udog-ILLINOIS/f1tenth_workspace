#!/usr/bin/env bash
# Launch the AutoDRIVE RoboRacer simulator (2026-iros practice build + TrackSelect plugin), from anywhere.
#   ./autodrive.sh fetch        download the multi-track app from our AutoDRIVE fork's release
#                               (built by autodrive/track_builder, branch f1tenth_multitrack)
#   ./autodrive.sh sim          1280x720 window, "Very Low" quality (higher sensor rate), Porto
#   ./autodrive.sh sim --full   the app's own resolution/quality settings
#   ./autodrive.sh sim --track NAME   started with -track NAME. Built in: porto (default), berlin,
#       srl2024cdc, srl2024iros, srl2025icra, srl2025cdctf; custom: iros2026
# The ROS 2 side lives in the ForzaETH container: see ./forzaeth.sh and docs/forzaeth_autodrive.md.
# The devkit (autodrive/devkit, branch AutoDRIVE-Devkit of our AutoDRIVE fork) is kept for
# reference; forzaeth.sh uses planners/forzaeth/autodrive_forzaeth's fast_bridge instead.
set -euo pipefail
root="$(cd "$(dirname "$0")" && pwd)"
dir="$root/autodrive/simulator_multitrack"
app="$dir/AutoDRIVE Simulator.app"
release=https://github.com/Udog-ILLINOIS/AutoDRIVE/releases/download/f1tenth_multitrack_v1/autodrive_simulator_multitrack_macos.zip

case "${1:-}" in
  fetch)
    [ -d "$app" ] && { echo "Already installed: $app" >&2; exit 0; }
    mkdir -p "$dir"; tmp="$(mktemp -d)"
    curl -fL -o "$tmp/sim.zip" "$release"
    ditto -x -k "$tmp/sim.zip" "$dir"; rm -rf "$tmp"
    xattr -cr "$app"; echo "Installed $app" ;;
  sim)
    shift; full=0; track=()
    while [ $# -gt 0 ]; do
      case "$1" in
        --full) full=1; shift ;;
        --track) track=(-track "$2"); shift 2 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
      esac
    done
    [ -d "$app" ] || { echo "Simulator not found at $app (run: $0 fetch)" >&2; exit 1; }
    # -n: open a new instance even if another copy of the sim is running
    # The sim sends one sensor frame per rendered frame, so a small low-quality window
    # gives ForzaETH a faster control loop.
    if [ $full = 1 ]; then exec open -n -a "$app" --args ${track[@]+"${track[@]}"}; fi
    exec open -n -a "$app" --args -screen-fullscreen 0 -screen-width 1280 -screen-height 720 \
      -screen-quality "Very Low" ${track[@]+"${track[@]}"} ;;
  *)
    echo "usage: $0 fetch | sim [--full] [--track NAME]" >&2; exit 2 ;;
esac
