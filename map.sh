#!/usr/bin/env bash
# Map the sim's current track and make the new map that track's default (what ./go.sh races).
#   ./map.sh [TRACK] [map_track.sh args...]
#   ./map.sh                       wall follower drives 2 laps on the sim's current track
#   ./map.sh berlin --laps 3       switch the sim to berlin first; 3 laps
#   ./map.sh --gap-follow          follow the gap drives instead (experimental); --manual: drive by hand
#                                  (ros2 run autodrive_bridge teleop_keyboard in the container)
# Needs ./getup.sh first (container, bridge, connected sim). The map goes to tracks/<track>_<date>_<time>/
# (planners/sigrobotics/ws/tracks); if it gets a clean mintime raceline, tracks/<track> becomes a symlink
# to it. A fallback raceline asks first; a failed raceline or mapping keeps the old default and prints a
# REVERTED banner (with a terminal bell). The first time, the old
# tracks/<track>/ folder is renamed to tracks/<track>_<date of its map>/ (nothing is deleted).
set -euo pipefail
root="$(cd "$(dirname "$0")" && pwd)"
compose=(docker compose -f "$root/planners/sigrobotics/docker-compose.yml")
tracks="$root/planners/sigrobotics/ws/tracks"

"$root/autodrive.sh" ctl status 2>/dev/null | grep -q '"connected":true' \
  || { echo "sim not connected to a bridge: run ./getup.sh first" >&2; exit 1; }
[ -t 0 ] || { echo "map.sh needs a terminal (map_track.sh reads Enter presses)" >&2; exit 1; }

track=
if [ $# -gt 0 ] && [[ "$1" != -* && "$1" != *:=* ]]; then track=$1; shift; fi
follow=--wall-follow; args=()
for a in "$@"; do
  case "$a" in --wall-follow|--gap-follow) follow=$a ;; --manual) follow= ;; *) args+=("$a") ;; esac
done
sim_track=$(python3 "$root/autodrive/map_feed.py" --current)
[ -n "$track" ] || track=$sim_track
if [ "$track" != "$sim_track" ]; then
  "$root/autodrive.sh" ctl tracks | grep -q "\"name\":\"$track\"" \
    || { echo "the sim has no track '$track' (sim is on $sim_track)" >&2; exit 1; }
  echo "== switching the sim from $sim_track to $track"
  "$root/autodrive.sh" ctl track "$track" >/dev/null; sleep 3
fi

name=${track}_$(date +%Y%m%d_%H%M)
echo "== mapping $track as $name${follow:+ ($follow)}"
status=0
"${compose[@]}" exec ros bash -c 'exec scripts/map_track.sh "$@"' map "$name" ${follow:+"$follow"} \
  ${args[@]+"${args[@]}"} || status=$?

current() {   # what tracks/<track> races now
  if [ -L "$tracks/$track" ]; then readlink "$tracks/$track"
  elif [ -e "$tracks/$track" ]; then echo "$track/ (the original folder)"
  else echo "nothing (no map for $track yet)"; fi
}
alert() {   # loud banner + terminal bell: the new map did NOT become the default
  printf '\a\n' >&2
  printf '%s\n' "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" \
    "!!  $1" "!!  REVERTED: $track still races $(current)" "!!  $2" \
    "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" >&2
  printf '\a' >&2
}

if [ "$status" -eq 1 ] || [ ! -f "$tracks/$name/raceline.csv" ]; then
  if [ -f "$tracks/$name/map.pbstream" ]; then
    alert "RACELINE FAILED for $name (map saved, no raceline; see build_raceline above)" \
      "Retry the raceline: docker compose -f planners/sigrobotics/docker-compose.yml exec ros bash -lc 'source install/setup.bash; ros2 run planning build_raceline $name'"
  else
    alert "MAPPING FAILED for $name (no map saved; see the errors above)" "Run ./map.sh again"
  fi
  exit 1
fi
if [ "$status" -eq 2 ]; then
  printf '\a'
  read -rp "$name has a FALLBACK raceline (not clean mintime, see above). Make it the default for $track anyway? [y/N] " ans
  if [[ "$ans" != [yY]* ]]; then
    alert "RACELINE NOT CLEAN for $name (fallback line, not TUM mintime)" "Race the new map anyway with: ./go.sh $name"
    exit 2
  fi
fi

link="$tracks/$track"
if [ -e "$link" ] && [ ! -L "$link" ]; then   # first time: keep the old folder under a dated name
  old=${track}_$(date -r "$link/map.pbstream" +%Y%m%d_%H%M 2>/dev/null || date -r "$link" +%Y%m%d_%H%M)
  [ ! -e "$tracks/$old" ] || old=${old}_old
  mv "$link" "$tracks/$old"
  echo "== old tracks/$track moved to tracks/$old"
fi
prev=$(readlink "$link" 2>/dev/null || true)
ln -sfn "$name" "$link"
echo "== tracks/$track -> $name (default for ./go.sh $track)${prev:+; was $prev}"
echo "   (the Mapping tab still shows the old map until the map feed is restarted: Ctrl-C ./getup.sh, run it again)"
exit "$status"
