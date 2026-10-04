# ForzaETH race stack on AutoDRIVE

The [ForzaETH race stack](https://github.com/ForzaETH/race_stack) (ROS 2 `ros2-humble`
branch, fresh clone 2026-10-04) racing in the AutoDRIVE **RoboRacer** simulator
(release `2026-iros`, practice build). Every run is logged (laps, speed, crashes) so global
racelines can be compared, and **every crash resets the car to the start line** over the API.

```
AutoDRIVE app (macOS) --Socket.IO :4567--> fast_bridge --> /autodrive/roboracer_1/*  (+ /autodrive/reset_command)
                                                                 |
                                                              adapter  --> /car_state/{scan,odom,pose}, /scan, /vesc/sensors/imu/raw, TF
                                                                 ^
ForzaETH: global planner (offline) -> state machine (GB_TRACK) -> MAP controller --/drive--+
run_logger --> data/[<experiment>/]<map>_<line>_x<speed>_<time>/    (crash -> /autodrive/reset_command)
```

What is ForzaETH and what is ours: ForzaETH plans the global line (offline, from the map),
its state machine picks the waypoints, and its MAP controller produces every drive command
(it is the only publisher on `/drive`). Ours is plumbing: the bridge and adapter translate
AutoDRIVE's sensors and commands, the mapper builds the map, and the logger records and
resets. In time trials ForzaETH does no online planning. Its local planner only overtakes
other cars.

Everything ROS runs in one container, `forzaeth_autodrive`. AutoDRIVE runs natively.

## Layout

| Path | What |
|---|---|
| `forzaeth.sh` | the only entry point you need (`./forzaeth.sh` alone prints usage) |
| `autodrive.sh`, `autodrive/` | the multi-track simulator app (`autodrive/simulator_multitrack/`, `./autodrive.sh fetch`), its source (`autodrive/track_builder/`) and the devkit (`autodrive/devkit/`, reference only), both submodules of our AutoDRIVE fork |
| `planners/forzaeth/race_stack/` | upstream clone, `ros2-humble`, two local patches (below); container `~/ws/src/race_stack` |
| `planners/forzaeth/autodrive_forzaeth/` | our package: bridge, adapter, mapper, run logger, raceline and analysis scripts; container `~/ws/src/autodrive_forzaeth` |
| `planners/forzaeth/build_cache/humble/` | colcon build/install/log (mounted at `~/ws/{build,install,log}`) |
| `racelines/<map>/` | raceline library: `<map>_<line>.json` (+ `<map>_<line>.csv` source), `<map>_track.csv`, `ACTIVE` (short line name); container `~/ws/racelines` |
| `data/` | logged runs, one folder per experiment (`data/raceline_comparison/`, ...); container `~/ws/runs`. `data/archive/` holds old runs and is skipped by `compare` unless you pass it |
| `docker/forzaeth.compose.yml`, `docker/forzaeth_autodrive.Dockerfile` | container: upstream `race_stack:humble_arm` + bridge deps + Foxglove |
| `planners/forzaeth/race_stack/stack_master/maps/autodrive_roboracer/` | the track: map, `global_waypoints.json`, speed scaling (ForzaETH reads maps from here) |

Names: a line is `<track>_<line>` (e.g. `autodrive_roboracer_min_curvature`), a run is
`<track>_<line>_x<speed>_<YYYYmmdd_HHMMSS>`. `forzaeth.sh` commands take the short line name
(`--line min_curvature`, `line use autodrive_roboracer centerline`); the track prefix is added for you.

## Every session

```bash
./autodrive.sh sim          # 1280x720, "Very Low" quality (faster sensor rate)
./forzaeth.sh up
./forzaeth.sh bridge        # then in the sim: connect to 127.0.0.1:4567 and switch to autonomous
./forzaeth.sh race --line forza_iqp --speed 0.5 --laps 5 [--experiment raceline_comparison]
./forzaeth.sh compare       # every run under data/ except data/archive/; or: compare data/raceline_comparison
```

Leave the bridge running all session. Restarting it drops the sim's connection, and you then
have to reconnect in the sim. Everything else (`race`, `map`, `plan`, `stop`, `reset`)
restarts freely.

## A race, step by step

`./forzaeth.sh race --line L --speed S --laps N` does the following:

1. Makes `L` the active line and sets ForzaETH's speed scaling to `S`.
2. Resets the car to the start line.
3. Starts ForzaETH: the base system, then the controller and state machine.
4. Logs N laps and stops on its own.

- **Lap 1 is a standing start.** Timing begins when the car moves. The laps after it are
  flying laps. Best and mean lap time use flying laps when there are any.
- **Crashes** come from the sim's own collision counter. A car stuck for 1.5 s while commanded
  to move counts too. Each crash:
  1. resets the car to the start line,
  2. logs the interrupted lap as `aborted` (invalid),
  3. starts the next lap as a standing start.
- `--laps` counts lap attempts, aborted ones included, so a line that always crashes still
  finishes.

## Trying a new global line

```bash
./forzaeth.sh line export autodrive_roboracer          # -> racelines/autodrive_roboracer/autodrive_roboracer_track.csv
# optimize on that track.csv with any tool in raceline_optimization/ (TUM format: x_m,y_m,w_tr_right_m,w_tr_left_m)
./forzaeth.sh line import autodrive_roboracer path/to/traj_race_cl.csv my_line
#   -> racelines/autodrive_roboracer/autodrive_roboracer_my_line.json (+ the CSV copied to ..._my_line.csv)
./forzaeth.sh race --line my_line --speed 0.5 --laps 5
./forzaeth.sh line list autodrive_roboracer            # * marks the active line
```

- `track.csv` widths are the raw distances to the walls, with no safety margin. Add your
  own (the car is 0.30 m wide; ForzaETH plans with `safety_width` 0.7 m).
- Import takes TUM `traj_race_cl.csv` (`s; x; y; psi; kappa; vx; ax`, psi 0 = north) or a
  plain `x,y[,vx]` CSV (`--v-const` if there's no speed). It recomputes s, heading,
  curvature and the distances to the walls. It refuses a line that runs against the track
  direction and warns about points outside the mapped track.
- `--speed` scales the line's own speed profile. Compare lines at the same `--speed`.
- The library starts with ForzaETH's own lines: `forza_iqp` (iterative min curvature,
  est. 5.17 s at 1.0x, 8.8 m/s) and `forza_sp` (shortest path, est. 5.34 s).

## What gets logged

`data/[<experiment>/]<map>_<line>_x<speed>_<YYYYmmdd_HHMMSS>/` (`<map>_<line>_x<speed>` is the
run label, also `raceline` in `summary.json`; `track`, `line`, `speed_scaling` are separate fields):

| File | Content |
|---|---|
| `laps.csv` | per lap: time, valid, start (`standing`/`flying`/`aborted`), crashes, max/mean speed, closest wall, distance, the sim's own lap time |
| `events.csv` | each crash: time, lap, type (`collision`/`stuck`), x, y, speed, whether it reset |
| `telemetry.csv` | 20 Hz: pose, speed, commanded speed and steering, closest wall |
| `summary.json` | best/mean/std lap time, totals, all laps and crash events |
| `metadata.json`, `global_waypoints.json`, `speed_scaling.yaml`, `raceline_source.csv` | snapshot of what was driven (ForzaETH commit/patches, exact waypoints, speed scaling, source CSV) |

`scripts/analyze_run.py ~/ws/runs/<experiment>/<run>` (run it in the container; map and speed come from the run) reports how closely a run tracked
the line: lateral error, actual vs commanded yaw rate, and achieved vs demanded lateral
acceleration. ForzaETH's own `lap_analyser` also runs (logs in the container at
`~/ws/data/lap_analyser/`).

## The simulator

AutoDRIVE RoboRacer `2026-iros` practice build, installed 2026-10-04 (it replaced Simulator
0.3.0 from 2023, which had no reset, collision or lap data over the API). Measured:

| | |
|---|---|
| Rate | 32–45 frames/s at 1280x720 Very Low. The sim sends one frame per rendered frame, so this is the control loop rate |
| Throttle | acts as a speed setpoint, about 25 m/s per unit (0.05→1.25, 0.2→4.92 m/s). The adapter commands `v_ref/25` plus a small PI trim |
| Steering | linear, command ±1 → ±0.524 rad; the servo slews at about 3.3 rad/s |
| Speed | the sim's own body-frame velocity (`odom` twist) |
| LiDAR | 1081 beams, ±135°, 10 m, sent gzip-compressed; 0.2733 m ahead of the rear axle |
| Reset | `V1 Reset` puts the car back on the spawn (= our start/finish line) without dropping the connection |
| Also sent | lap count, lap time, last/best lap, collision count (published by the bridge) |

The map (`autodrive_roboracer`) was built by `./forzaeth.sh map` from the sim's
ground-truth pose plus the LiDAR, so it is in the sim's world frame and needs no
localization. The track is about 7 × 17 m and 1.06–2.46 m wide, with an S-bend at
y ≈ −2.

## Results (forza_iqp)

| Speed | Laps | Flying lap | Crashes |
|---|---|---|---|
| 0.3x | 3 flying, clean | 16.99–17.03 s | 0 |
| 0.5x | 3 flying, clean | best 10.05 s, mean 10.09 ± 0.03 s | 0 |
| 0.8x | 0 of 5 | – | 5 (collisions in the S-bend and at the hairpin; each reset the car) |

The 0.3x and 0.8x rows predate the sector_tuner fix below. They are in
`data/archive/before_sector_tuner_fix/`. At 0.5x the flying laps were the same before and
after the fix (10.05 s), so the bug seems to have hit mainly the first lap. Rerun those
speeds before trusting them. The planner's
line assumes ForzaETH's real car (`racecar_f110.ini`, ggv), not this sim, which is why it
has to be scaled down.

## Changes to upstream ForzaETH / gotchas

- **Patched `utilities/nodes/sector_tuner/sector_tuner/sector_tuner.py`** (upstream bug).
  `scale_points()` aliased the scaled waypoints to the originals and then scaled them in
  place on every 0.5 s timer tick. Any speed scaling below 1.0 therefore compounded (0.5x
  became 0.25x, 0.125x, …) until `/global_waypoints` was republished. That showed up as
  the car slowing to a stop for several seconds after the start. The fix is a deepcopy.
- **Patched `stack_master/setup.py`**: its `config/NUC5/*/*` glob matched a directory
  (`pacejka/dubi`) and broke the build. The patch filters that glob to files only.
- Upstream `time_trials_launch.xml` declares `LU_table` but never passes it, so the controller
  looks for a NUC5 table that isn't shipped. `race_launch.xml` forwards `SIM_linear`.
- The MAP controller also reads `/vesc/sensors/imu/raw` (longitudinal accel, VESC-rotated) and
  `/scan` (follow-the-gap fallback). The adapter publishes both.
- The global planner must run as node `global_planner` (its params file is keyed to that
  name). It finishes but never exits, so `forzaeth.sh plan` waits for the output file and
  then kills it.
- `f110_gym` / `f1tenth_gym_ros` aren't built (AutoDRIVE replaces them).
- RMW is FastDDS, not upstream's zenoh, because everything runs in one container.
- The TUM optimizer is pip-installed into the container from source. `forzaeth.sh up`
  reinstalls it if the container was recreated.
- Foxglove: `ws://localhost:8766` (8765 is the Humble sim's).
- `fast_bridge` replaces the devkit's ROS 2 bridge (same topics), dropping the per-frame
  camera decode and TF tree. It zeroes commands that are more than 0.5 s old and clears them
  on reset, so a stopped controller never leaves the car driving.
