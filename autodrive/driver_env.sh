# Sourced by getup.sh / go.sh: the container of driver N (car N of a multi-car sim, MultiCar.cs).
# Driver 1: the club stack container as always (host port 4567, ROS domain 0, compose project sigrobotics).
# Driver N: compose project sigrobotics_dN, host port 4566+N -> the bridge's 4567, Foxglove 8764+N, ROS domain N.
driver=${1:-1}
[[ "$driver" =~ ^[1-8]$ ]] || { echo "--driver must be 1-8" >&2; exit 2; }
compose_file="$root/planners/sigrobotics/docker-compose.yml"
if [ "$driver" = 1 ]; then
  compose=(docker compose -f "$compose_file"); driver_flag=
else
  export BRIDGE_PORT=$((4566 + driver)) FOXGLOVE_PORT=$((8764 + driver)) ROS_DOMAIN_ID=$driver
  # the club stack's compose file as it is, plus autodrive/driver_compose.yml (this driver's ports, ROS domain)
  compose=(docker compose -p "sigrobotics_d$driver" -f "$compose_file" -f "$root/autodrive/driver_compose.yml"); driver_flag=" --driver $driver"
fi
car_connected() {   # car $driver connected to its bridge (car 1: the sim's own socket)
  if [ "$driver" = 1 ]; then "$root/autodrive.sh" ctl status 2>/dev/null | grep -q '"connected":true'
  else "$root/autodrive.sh" ctl cars 2>/dev/null | python3 -c "import json,sys; c={x['car']:x for x in json.load(sys.stdin)['cars']}; sys.exit(0 if c.get($driver,{}).get('connected') else 1)"
  fi
}
