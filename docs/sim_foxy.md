# Simulator: ROS 2 Foxy (`f1tenth_gym_ros`, branch `main`)

Setting up and launching the **simulator only**. Lab instructions live in each lab's own folder
(`labs/lab1_ros_basics/LAUNCH.md`, `labs/lab2_emergency_braking/LAUNCH.md`, ...), and they assume this stack is up.

| | |
|---|---|
| Sim branch | `f1tenth_gym_ros` on `main` (Foxy) |
| Compose project | `f1tenth_gym_ros_foxy` |
| Containers | `f1tenth_gym_ros_foxy-sim-1` (ROS 2 Foxy + sim), `f1tenth_gym_ros_foxy-novnc-1` (browser display) |
| Image | `f1tenth_gym_ros:foxy` (built `FROM ros:foxy`, arm64 on this Mac) |
| Viewer | RViz through noVNC: <http://localhost:8080/vnc.html> (click **Connect**) |
| Sim workspace | `/sim_ws` (source `/sim_ws/install/local_setup.bash`) |
| Managed with | `./sim.sh foxy <docker-compose args>` from the project root |

Everything below runs from the project root unless noted:

```bash
cd ~/Desktop/University_of_Illinois/SigRobotics/F1Tenth
```

`sim.sh` adds the compose settings, and refuses to start the stack if `f1tenth_gym_ros` isn't on `main` or if another stack is using port 8080.
It also mounts the lab packages into the sim container (`docker/foxy_labs.compose.yml`); the labs' launch docs explain how to use them.

---

## 1. Start Docker Desktop

```bash
open -a Docker
until docker info >/dev/null 2>&1; do sleep 2; done; echo "docker is up"
```

## 2. Get on the Foxy branch

```bash
git -C gym_ros_workspace/f1tenth_gym_ros checkout main
```

## 3. Start the containers

First time ever (or if `docker images f1tenth_gym_ros:foxy` shows nothing): build the image. It takes several minutes.

```bash
./sim.sh foxy up -d --build
```

Every time after that:

```bash
./sim.sh foxy start        # resume the stopped containers (keeps anything built inside them)
# or
./sim.sh foxy up -d        # create/refresh them; recreates the sim container if the compose config or mounts changed
```

Check: `./sim.sh foxy ps` should show both containers `Up`.

## 4. Open a shell in the sim container

```bash
docker exec -it f1tenth_gym_ros_foxy-sim-1 bash
```

In **every** new shell inside the container:

```bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash
```

`tmux` is installed (`Ctrl+B` then `%` splits a pane, `Ctrl+B` then an arrow key moves). Or open more Mac terminals and repeat this step.

## 5. Launch the simulator

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Leave it running. It starts the sim bridge, the map server and RViz.

## 6. View it

Open <http://localhost:8080/vnc.html> and click **Connect**. RViz should show the Levine map, the car and the lidar scan.

- **Not visually confirmed on this setup.** The web page serves and `rviz2` runs, but RViz logs a GLSL shader error for the map display and
  repeated `Invalid frame ID "map"` warnings. The `map -> ego_racecar/base_link` transform does resolve. If the map is missing, that's RViz's software rendering, not the sim.

## 7. Drive and reset

Keyboard teleop (`kb_teleop` is `True` in `f1tenth_gym_ros/config/sim.yaml`), in a second sourced shell:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`i` forward · `,` back · `u` / `o` forward + turn · `m` / `.` back + turn · `k` stop · `q` / `z` speed +/-10%. Keep the teleop terminal focused.

Reset the car: in RViz use **2D Pose Estimate** (click a position, drag for heading), or from a shell (this example: `x=8, y=0`, facing west):

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 8.0, y: 0.0}, orientation: {z: 1.0, w: 0.0}}}}"
```

## What the sim looks like (measured here)

- Topics: `/scan`, `/ego_racecar/odom`, `/drive` (`AckermannDriveStamped`, the sim listens here), `/cmd_vel` (teleop), `/initialpose` (reset), `/map`, `/tf`.
- `/scan` and `/ego_racecar/odom` each publish at about 240 Hz. The scan has 1080 beams over 270 degrees, range 0 to 30 m.
- Default start pose is `(0, 0)` facing east, inside a Levine hallway about 1.7 m wide.
- The sim's requested speed is whichever of `/drive` or `/cmd_vel` arrived last.
- **Walls are not solid.** The car drives straight through them; the sim only flags a collision.

Config (`f1tenth_gym_ros/config/sim.yaml`, edited on the Mac, applied on the next launch): map, start poses, `num_agent`, `kb_teleop`, topic names.

## Stop, resume, remove

- `Ctrl+C` stops a launch or node. `exit` leaves a shell without stopping the container.
- Done for now: `./sim.sh foxy stop`. Resume later with `./sim.sh foxy start`.
- Remove the containers (images kept; anything built inside them is lost): `./sim.sh foxy down`.

## Switch to Humble

```bash
./sim.sh foxy stop
git -C gym_ros_workspace/f1tenth_gym_ros checkout dev-humble
```

Then follow `sim_humble.md`. Run one stack at a time: `sim.sh` refuses a wrong-branch start and a start while the other stack holds port 8080.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `Cannot connect to the Docker daemon` | Start Docker Desktop (step 1). |
| `f1tenth_gym_ros is on '...', but the foxy stack needs 'main'` | `git -C gym_ros_workspace/f1tenth_gym_ros checkout main`. |
| `Port 8080 is in use by: ...` | Another stack is running. Stop it (`./sim.sh humble stop`). |
| No `/scan` or odom | The sim isn't launched (step 5), or you forgot the two `source` lines. |
| RViz map missing in the browser | Known RViz GLSL issue under software rendering; the car and scan should still draw. |
| `docker: container name ... already in use` | Don't use `docker run`; use `./sim.sh foxy up -d` or `start`. |
| `no matching manifest` / very slow | Apple Silicon: the noVNC image is amd64 and runs under emulation (a warning is normal). |
