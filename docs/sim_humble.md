# Simulator: ROS 2 Humble (`f1tenth_gym_ros`, branch `dev-humble`)

**This is now the race stack.** Labs are mounted (same 6 packages as Foxy), the real `levine_blocked`
race map is loaded with a working centerline, and `/ego_racecar/lap_count` / `/lap_time` publish for
real — none of that exists on the Foxy stack (see "Why not just race on Foxy" below).

| | |
|---|---|
| Sim branch | `f1tenth_gym_ros` on `dev-humble` |
| Compose project | `f1tenth_gym_ros_humble` |
| Containers | `f1tenth_gym_ros_humble-sim-1` (plus `-novnc-1`, only if you use the RViz fallback) |
| Image | `f1tenth_gym_ros:humble` (built `FROM ros:humble`, about 4 GB) |
| Physics engine | `f1tenth_gym` 1.0.0.dev0 (`/sim_ws/f1tenth_gym`, the `dev-humble` GYM_REF — **not** the JAX/`dev-jax` engine Jazzy uses) |
| Viewer | **Foxglove** (desktop app or web) at `ws://localhost:8765` — no RViz/noVNC by default |
| Sim workspace | `/sim_ws` (source `/opt/ros/humble/setup.bash` then `/sim_ws/install/local_setup.bash`) |
| Managed with | `./sim.sh humble <docker-compose args>` from the project root |

```bash
cd ~/Desktop/University_of_Illinois/SigRobotics/F1Tenth
```

`sim.sh` refuses to start Humble if `f1tenth_gym_ros` isn't on `dev-humble` or if the Foxy stack is still using port 8080.

---

## Why not just race on Foxy

Foxy's `main` branch runs the *old* engine (`f110-gym` 0.2.1, `gym`-based API). Lap counting is computed
inside the engine from a track centerline — `f110_env.py`'s `lap_counts`/`lap_times` — and that whole
mechanism plain doesn't exist in the old engine, at any config. `dev-jazzy` has it, but only by also
swapping to a different, JAX-based `f1tenth_gym` package (`dev-jax` branch) and Python 3.12 — a real
engine-version jump, not something to reproduce on Foxy. `dev-humble` turned out to already carry the
*same new engine* `dev-jazzy` uses (just not the JAX variant, and without the ROS-topic wiring for laps)
— confirmed by direct import inside the built image, not assumed. That's what makes Humble the practical
middle ground: real lap counting, without adopting the JAX/Jazzy engine.

## 1. Stop Foxy, start Docker, switch branch

```bash
./sim.sh foxy stop
open -a Docker
until docker info >/dev/null 2>&1; do sleep 2; done; echo "docker is up"
git -C gym_ros_workspace/f1tenth_gym_ros checkout dev-humble
```

`main` (Foxy) may have uncommitted changes — see **"Foxy's parked changes"** below before switching if you
last worked on Foxy.

## 2. Start the container

First time (or if `docker images f1tenth_gym_ros:humble` shows nothing): build it. It pulls `ros:humble`
and takes several minutes.

```bash
./sim.sh humble up -d --build
```

Afterwards (or any time compose config — e.g. mounts — changed, `up -d` recreates as needed):

```bash
./sim.sh humble up -d
```

Check: `./sim.sh humble ps`. The sim container should be `Up` with port `8765` published, and
`docker exec f1tenth_gym_ros_humble-sim-1 ls /sim_ws/src/` should list `f1tenth_gym_ros` plus all 6 lab
packages (`lab1_pkg`, `safety_node`, `wall_follow`, `gap_follow`, `pure_pursuit`, `lab7_pkg`, `mpc`).

## 3. Build and launch the sim

```bash
docker exec -it f1tenth_gym_ros_humble-sim-1 bash
source /opt/ros/humble/setup.bash
cd /sim_ws && colcon build --packages-select f1tenth_gym_ros
source /sim_ws/install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Leave it running. You'll see one harmless error in the log — `[xdg-open-2] FileNotFoundError:
'xdg-open'` — the launch file tries to open a browser inside the headless container; ignore it,
`foxglove_bridge` starts fine independently.

Rebuild `f1tenth_gym_ros` (not just the lab package) any time you touch `config/sim.yaml`,
`maps/*.yaml`, `maps/*_centerline.csv`, or `f1tenth_gym_ros/gym_bridge.py` — the install space is a
copy, not a live link, and `ros2 launch` reads from it.

## 4. Connect Foxglove and pin the chase-cam view

```bash
open "foxglove://open?ds=foxglove-websocket&ds.url=ws://localhost:8765"
```

(Or manually: **Open connection → Foxglove WebSocket**, URL `ws://localhost:8765`, **Open**. Web version:
<https://app.foxglove.dev/?ds=foxglove-websocket&ds.url=ws://localhost:8765>, Chrome only.)

Then, **once**, load the chase-cam layout (a plain `.json` file — Foxglove doesn't register a file
association for it, so this is a manual import, not something scriptable from outside the app):

**Layout menu → Import from file... → `f1tenth_gym_ros/config/foxglove/gym_bridge_foxglove.json`.**

That layout's 3D panel already ships with `followMode: follow-pose` / `followTf: ego_racecar/base_link`
— the camera tracks the car automatically once it's loaded, no per-session setup after that.

Confirmed working with Foxglove 3.2.1 (desktop) / `foxglove_bridge` 3.5.0. A much older Foxglove app can
fail the handshake (`foxglove.sdk.v1` subprotocol).

## 5. Race a lab

Build the lab package once, then run it (needs its own sourced shell alongside the sim's):

```bash
docker exec -it f1tenth_gym_ros_humble-sim-1 bash
source /opt/ros/humble/setup.bash
cd /sim_ws && colcon build --packages-select <pkg>
source /sim_ws/install/local_setup.bash
ros2 run <pkg> <executable>
```

| Lab | `<pkg>` | `<executable>` |
|---|---|---|
| 1 — ROS basics | `lab1_pkg` | launch file, not a single node: `ros2 launch lab1_pkg lab1_launch.py` |
| 2 — Emergency braking | `safety_node` | `safety_node` |
| 3 — PID wall following | `wall_follow` | `wall_follow_node` — **timed here**: 4 clean laps, zero collisions, ~50.4 s mean lap (see `race_results.md`) |
| 4 — Follow the Gap | `gap_follow` | `reactive_node` — **timed here**: 4 clean laps, zero collisions, ~123.6 s mean lap (see `race_results.md`) |
| 5 — Pure pursuit | `pure_pursuit` | `pure_pursuit_node.py` |
| 6 — Motion planning (RRT) | `lab7_pkg` | `rrt_node.py` (Python, implemented) |
| 8 — MPC | `mpc` | `mpc_node.py` (needs `cvxpy`+`osqp`) |

Watch laps complete either in the launch terminal's log (`ego_racecar completed lap N, last lap X.XX s`)
or on the topics directly:

```bash
ros2 topic echo /ego_racecar/lap_count   # Int32, increments per completed lap
ros2 topic echo /ego_racecar/lap_time    # Float32, seconds, updates when a lap completes
```

Reset the car to the default start pose `(-12, 0)` facing east, or anywhere else, from a sourced shell:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: -12.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

## What was actually changed to make racing work here (2026-09-24)

Three things, all on `dev-humble`, none of it upstream — worth knowing if `git status` in
`f1tenth_gym_ros` looks unfamiliar:

1. **`config/sim.yaml`**: `map_path` switched from `maps/levine` to `maps/levine_blocked` (the real,
   sealed-doorway race map).
2. **Centerline data**: `maps/levine_centerline.csv` copied in from upstream `dev-jazzy` (it's the exact
   file the real autograder's own `make_centerline.py` produces — 499 pts, 61.78 m, counter-clockwise —
   reused rather than regenerated) and copied again as `levine_blocked_centerline.csv` /
   `levine_obs_centerline.csv`, because our installed engine's loader looks for a file literally named
   `<map-stem>_centerline.csv` next to each map's `.yaml` (it does **not** understand a `centerline:` key
   *inside* the yaml — that parsing is Jazzy-only code, not present here — adding one there breaks
   `TrackSpec` with an unexpected-kwarg error). Also had to strip a second descriptive `#` comment line
   the source file had; the installed `Raceline.from_centerline_file` only tolerates one header line.
3. **`f1tenth_gym_ros/gym_bridge.py`**: ported just the lap-count/lap-time ROS publishing (~30 lines —
   `Int32`/`Float32` on `/ego_racecar/lap_count` and `/lap_time`, plus the per-opponent equivalents) from
   the `dev-jazzy` commit that introduced it. Nothing else from that commit — not the JAX pin, not the
   `num_agent`→`num_agents` rename, not the rear-axle pose change — was pulled in; the engine already
   computed `lap_counts`/`lap_times` internally on `dev-humble` without any of that (confirmed by reading
   `/sim_ws/f1tenth_gym/f1tenth_gym/envs/f110_env.py` inside the built image), so only the publisher was
   missing.

`docker/humble.compose.yml` also now mounts all 6 lab packages, mirroring `docker/foxy_labs.compose.yml`'s list (that
file itself is untouched).

## What Humble looks like (measured here)

- Same topic names as Foxy: `/scan`, `/ego_racecar/odom`, `/drive`, `/cmd_vel`, `/initialpose`. Extras:
  `/ego_racecar/collision` (`std_msgs/Bool`), `/ego_racecar/lap_count` (`Int32`), `/ego_racecar/lap_time`
  (`Float32`), `/pause_sim`, `/diagnostics`.
- `/scan` and odom each publish at about 250 Hz.
- Default start pose is **`(-12, 0)`** facing east (Foxy starts at `(0, 0)`). Real `levine_blocked` map —
  sealed doorways, same footprint as plain Levine otherwise.
- Nodes launched: `bridge`, `map_server`, `foxglove_bridge`, `lifecycle_manager_localization`,
  `ego_robot_state_publisher`. **No RViz** by default.

## Foxy's parked changes

`main` had its own uncommitted work (`levine_blocked` map switch + RViz chase-cam view) when this Humble
setup started — stashed rather than lost or carried over:
`git -C gym_ros_workspace/f1tenth_gym_ros stash list` shows it as `foxy(main): levine_blocked map switch + rviz chase-cam,
pre-humble-switch`. Restore it when you next work on Foxy: `git checkout main && git stash pop`.

## RViz fallback (noVNC), optional and untested here

```bash
./sim.sh humble --profile novnc up -d          # also starts the noVNC container (port 8080)
# in the sim container, after sourcing:
rviz2 -d src/f1tenth_gym_ros/config/rviz/gym_bridge.rviz
```

Then open <http://localhost:8080/vnc.html>. The `novnc` service is behind a Compose profile in Humble, so
a plain `up` doesn't start it.

## Stop, resume, remove

- Done for now: `./sim.sh humble stop`. Resume: `./sim.sh humble start`. Remove the containers (images
  kept): `./sim.sh humble down`.
  (`sim.sh` includes the noVNC profile for these, so a running noVNC container isn't left holding port 8080.)

## Switch back to Foxy

```bash
./sim.sh humble stop
git -C gym_ros_workspace/f1tenth_gym_ros checkout main
./sim.sh foxy start
```

Then follow `sim_foxy.md`. Remember the stash above if you want `levine_blocked`/the RViz chase-cam back
on Foxy too.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `f1tenth_gym_ros is on '...', but the humble stack needs 'dev-humble'` | `git -C gym_ros_workspace/f1tenth_gym_ros checkout dev-humble`. |
| `Port 8080 is in use by: ...` | The Foxy stack is running: `./sim.sh foxy stop`. |
| Foxglove can't connect | Sim not launched (step 3), or the app is too old for `foxglove.sdk.v1`; try the web app. |
| Camera doesn't follow the car | The chase-cam layout hasn't been imported this session — see step 4; it doesn't persist across a fresh Foxglove connection on its own. |
| Camera follows, but zoomed way out / at an angle, whole track visible | The layout's `cameraState` was originally saved zoomed out (`distance: 20`) and rotated off-axis (`thetaOffset: 45°`, plus a ~9m `targetOffset` dragging the look-at point away from the car). Fixed 2026-09-24 to `distance: 6`, `phi: 65`, `thetaOffset: 0`, `targetOffset: [0,0,0]` — a tight view centered directly behind the car. Re-import the layout (step 4) to pick up the fix; if it's still off to taste, `distance` and `phi` in `config/foxglove/gym_bridge_foxglove.json` → `cameraState` are the two to hand-tune (lower `distance` = closer, higher `phi` = more level/less top-down). |
| `Map has no centerline/raceline; disabling frenet frame and lap counting` in the bridge log | A `<map-stem>_centerline.csv` is missing next to that map's `.yaml`, or `f1tenth_gym_ros` wasn't rebuilt after adding one. |
| `TypeError: TrackSpec.__init__() got an unexpected keyword argument 'centerline'` | A map yaml has a `centerline:` key in it — remove it; use the `<map-stem>_centerline.csv` filename convention instead (see "What was actually changed" above). |
| `ros2 topic echo`/`list` fails with `xmlrpc.client.Fault: ... RuntimeError:!rclpy.ok()` | The `ros2` CLI daemon is in a stuck state (seen after force-killing sim processes). Fix: `ros2 daemon stop && ros2 daemon start`. |
| `[xdg-open-2]` error in the launch log | Harmless — the container can't open a browser; `foxglove_bridge` itself is unaffected. |
| Nothing draws in noVNC | Expected: Humble has no RViz by default; use Foxglove (or the RViz fallback above). |
| `Cannot connect to the Docker daemon` | Start Docker Desktop. |
