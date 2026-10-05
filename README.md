# F1Tenth

SIGRobotics F1Tenth workspace. Simulator stacks, labs, the AutoDRIVE simulator, and raceline optimization.

## Layout

```
F1Tenth/
  README.md                 this file: index + how to launch the sim stacks
  sim.sh                    Foxy/Humble docker-compose launcher
  autodrive.sh              AutoDRIVE RoboRacer simulator launcher
  forzaeth.sh               ForzaETH race stack on AutoDRIVE: map, plan, swap racelines, logged races
  docker/                   compose files: sim.sh's lab mounts per stack, forzaeth.compose.yml
  docs/                     sim_foxy.md, sim_humble.md, race_results.md, autodrive.md, forzaeth_autodrive.md
  gym_ros_workspace/        f1tenth_gym_ros workspace that sim.sh runs (Foxy and Humble)
    f1tenth_gym_ros/        simulator repo (branch picks Foxy vs Humble)
    labs/                   lab<N>_<topic>, each its own git repo, mounted into the sim container
  planners/
    forzaeth/               ForzaETH on AutoDRIVE (see docs/forzaeth_autodrive.md)
      race_stack/           ForzaETH race_stack clone, ros2-humble (maps in stack_master/maps/)
      autodrive_forzaeth/   our ROS 2 package: AutoDRIVE bridge/adapter, mapper, run logger, scripts
      build_cache/humble/   colcon build/install/log, mounted into the container
  racelines/<track>/        driven raceline library: <track>_<line>.json/.csv, <track>_track.csv, ACTIVE
  data/                     logged runs: <experiment>/<track>_<line>_x<speed>_<YYYYmmdd_HHMMSS>/, archive/
  autodrive/                multi-track sim app (local, ./autodrive.sh fetch), devkit/ + track_builder/ submodules (see docs/autodrive.md)
  raceline_optimization/    every global raceline generator + track data (see its README.md)
```

## Cloning

Third-party code lives in our forks under `Udog-ILLINOIS`, linked here as git submodules
(see `.gitmodules`). Each one tracks a branch of its fork. Full diagram:
[`docs/repo_structure.md`](docs/repo_structure.md). Keep it current: the hooks in `.githooks/` block a
commit or push when a folder or submodule isn't in it.

```bash
git clone --recurse-submodules https://github.com/Udog-ILLINOIS/f1tenth.git F1Tenth
git config core.hooksPath .githooks    # turn on the repo-structure check (commit + push)
git submodule update --remote          # pull the latest commit of each fork's tracked branch
git add <path> && git commit           # record the new pointer here
```

| Path | Fork (branch) |
|---|---|
| `autodrive/track_builder` | `AutoDRIVE` (`f1tenth_multitrack`): TrackSelect plugin + custom tracks; built app is a release |
| `autodrive/devkit` | `AutoDRIVE` (`AutoDRIVE-Devkit`, shallow) |
| `planners/forzaeth/race_stack` | `ForzaETH` (`ros2-humble`) |
| `gym_ros_workspace/f1tenth_gym_ros` | `f1tenth_gym_ros` (`dev-humble`) |
| `raceline_optimization/tum_optimizer` | `global_racetrajectory_optimization` (`master`) |
| `raceline_optimization/f1nn_init_shehadeh2026` | `f1-data-init-optimization` (`main`) |
| `raceline_optimization/track_data/*` | `racetrack-database`, `f1tenth_racetracks`, `f1tenth_maps` |

Not in git (see `.gitignore`): Python venvs, `planners/forzaeth/build_cache/`, the AutoDRIVE macOS apps
(`autodrive/simulator_multitrack/`, get it with `./autodrive.sh fetch`), `docs/F1Tenth_Papers/`,
and `gym_ros_workspace/labs/` (course labs stay local, so the lab links below only work on a local copy).

**ForzaETH on AutoDRIVE** (separate container, separate from the stacks below): see
[`docs/forzaeth_autodrive.md`](docs/forzaeth_autodrive.md). Quick start: `./autodrive.sh sim`,
`./forzaeth.sh up`, `./forzaeth.sh bridge` (click Connect + Autonomous in the sim), then
`./forzaeth.sh race --line forza_iqp --speed 0.3 --laps 5`.

# How to launch the sim stacks

Two separate stacks, one at a time. Both are driven by `./sim.sh` from this folder
(`~/Desktop/University_of_Illinois/SigRobotics/F1Tenth`).

| Stack | Sim branch | Doc | Viewer |
|---|---|---|---|
| **Foxy** | `main` | [`docs/sim_foxy.md`](docs/sim_foxy.md) | RViz via noVNC, <http://localhost:8080/vnc.html> |
| **Humble** (race stack — real `levine_blocked` + lap counting) | `dev-humble` | [`docs/sim_humble.md`](docs/sim_humble.md) | Foxglove, `ws://localhost:8765` |

Timed race results across labs live in [`docs/race_results.md`](docs/race_results.md), not scattered per-lab — each lab's
own `LAUNCH.md` links to it instead of repeating its numbers.

Both stacks mount all 6 lab packages now (as of 2026-09-24 — Humble didn't used to). Lab launch docs live
in each lab's folder and are written against the Foxy stack; for running the same lab on Humble instead
(to actually race it — Foxy's engine can't do lap counting at all, see `docs/sim_humble.md`'s "Why not just
race on Foxy"), swap in `docs/sim_humble.md`'s launch/build steps and use the executable straight from the
table below — the lab packages themselves don't change:

- [`gym_ros_workspace/labs/lab1_ros_basics/LAUNCH.md`](gym_ros_workspace/labs/lab1_ros_basics/LAUNCH.md): `talker` / `relay`
- [`gym_ros_workspace/labs/lab2_emergency_braking/LAUNCH.md`](gym_ros_workspace/labs/lab2_emergency_braking/LAUNCH.md): safety node in the simulator
- [`gym_ros_workspace/labs/lab3_pid_control/LAUNCH.md`](gym_ros_workspace/labs/lab3_pid_control/LAUNCH.md): PID wall following in the simulator (map caveat: no `levine_blocked`/lap count on Foxy `main`, see that doc; timed results in [`docs/race_results.md`](docs/race_results.md))
- [`gym_ros_workspace/labs/lab4_follow_gap/LAUNCH.md`](gym_ros_workspace/labs/lab4_follow_gap/LAUNCH.md): Follow the Gap reactive obstacle avoidance (map switching needs a rebuild; known open tuning issue on `levine_obs`; timed results in [`docs/race_results.md`](docs/race_results.md))
- [`gym_ros_workspace/labs/lab5_pure_pursuit/LAUNCH.md`](gym_ros_workspace/labs/lab5_pure_pursuit/LAUNCH.md): SLAM and Pure Pursuit — sim half only (adaptive lookahead + curvature-regulated speed tracking a centerline extracted from `levine_blocked`); Spielberg and the real-car SLAM/particle-filter parts aren't done, see that doc; timed results in [`docs/race_results.md`](docs/race_results.md) (Race 2)
- [`gym_ros_workspace/labs/lab6_motion_planning/LAUNCH.md`](gym_ros_workspace/labs/lab6_motion_planning/LAUNCH.md): Motion Planning (RRT local planner + occupancy grid) — package is `lab7_pkg` (upstream template's own numbering quirk, kept as-is); Python side implemented and raced (Race 2, `docs/race_results.md`), C++ side (`rrt.cpp`) still an unimplemented skeleton, RRT* extra credit not done, see that doc
- [`gym_ros_workspace/labs/lab8_mpc/LAUNCH.md`](gym_ros_workspace/labs/lab8_mpc/LAUNCH.md): Model Predictive Control — continues lab5's pure pursuit reference trajectory; fully implemented and raced (Race 2, `docs/race_results.md`), see that doc for the model, tuning, and the 9-bug chain it took to get clean laps

Lab 7 (Vision Lab) was set up and partly implemented (calibration, distance measurement,
lane detection — all verified) but skipped and deleted per the user's call: it's almost
entirely hardware-bound (real RealSense camera + Jetson + TensorRT), not simulator work.
Lab 8 is the last lab in the actual template series — `f1tenth_lab9_template` is a GitHub
redirect to the same repo as lab8, and `f1tenth_lab10_template` doesn't exist.

## Quick switch

```bash
# Foxy -> Humble                                   # Humble -> Foxy
./sim.sh foxy stop                                 ./sim.sh humble stop
git -C gym_ros_workspace/f1tenth_gym_ros checkout dev-humble         git -C gym_ros_workspace/f1tenth_gym_ros checkout main
./sim.sh humble start                              ./sim.sh foxy start
```

(First time on a stack: `./sim.sh <foxy|humble> up -d --build`. See that stack's doc.)

## Shared infrastructure changes (not lab-specific)

Changes below live in files shared across every lab (`gym_ros_workspace/f1tenth_gym_ros/`), so they affect whichever lab you're
running, not just the one open when the change was made. Called out here on their own, separately from any lab's
own instructions, so they don't get mistaken for part of a specific lab.

**RViz's default view now follows the car.** `f1tenth_gym_ros/launch/gym_bridge.rviz`'s `Views.Current` was changed:

| | Before | After |
|---|---|---|
| `Class` | `rviz_default_plugins/Orbit` | `rviz_default_plugins/ThirdPersonFollower` |
| `Target Frame` | `<Fixed Frame>` (resolves to the static `map` frame) | `ego_racecar/base_link` (the car) |
| `Distance` | `10` | `6` |

*Difference:* the camera now chases the car automatically instead of sitting fixed on the map's origin.
*Why:* `Orbit`'s view controller isn't built to track a moving target frame — pointing its `Target Frame` at the car
still leaves the camera drifting/zooming outward every TF update rather than following it. `ThirdPersonFollower`
(and `XYOrbit`, an alternative if you want to freely orbit/zoom while still tracking) are the RViz plugins actually
designed for a moving target. Since it's baked into the shared `.rviz` file, this took effect for every lab's viewer
going forward, not just the one that prompted it — it only needed a manual `rviz2` restart to show up in an
already-open window (a running RViz doesn't hot-reload its config from disk).

**Humble now races for real: `levine_blocked` + working lap counting.** `f1tenth_gym_ros` on `dev-humble`
had its `config/sim.yaml` map switched to `levine_blocked`, gained a centerline CSV per map (copied from
upstream `dev-jazzy`, the same file the real autograder's own generator produces), and `gym_bridge.py` got
a small ported lap-count/lap-time ROS publisher (not a full Jazzy engine swap — the installed `dev-humble`
engine already tracked laps internally, it just wasn't wired to a topic). Full detail, including exactly
why Foxy can't get this and what the Foxy analog of this change was, is in `docs/sim_humble.md`'s dedicated
section — not repeated here since it's Humble-only, not something every lab doc needs to restate.

## Notes on files

| File | What it is |
|---|---|
| `sim.sh` | Runs docker-compose for the chosen stack; refuses a wrong-branch start, or a start while the other stack holds port 8080. Reads its overrides from `docker/`. |
| `docker/foxy_labs.compose.yml` | Foxy stack settings: pinned image `f1tenth_gym_ros:foxy` and the lab package mounts. To add a lab, add one mount line. Relative paths inside resolve against `gym_ros_workspace/f1tenth_gym_ros/`, not `docker/`. |
| `docker/humble.compose.yml` | Humble stack settings: pinned image `f1tenth_gym_ros:humble` + the lab package mounts (mirrors the Foxy list). |
| `docs/race_results.md` | Timed lap results per lab on the Humble race stack — the roboracer-learn "Race 1" (labs 3-4) and "Race 2" (labs 5-6-8: pure pursuit, RRT, MPC) reference baselines, both complete. |
| `pure_pursuit/` (inside `lab5_pure_pursuit/`) / `lab7_pkg/` (inside `lab6_motion_planning/`) | Package folder names don't always match the lab number -- lab6's package is upstream's own `lab7_pkg`, kept as shipped rather than renamed (an autograder would expect the shipped name). |

Both compose files also mount `../races:/sim_ws/race` (that is `gym_ros_workspace/races/`), but that folder doesn't exist yet.

Containers are named `f1tenth_gym_ros_foxy-*` and `f1tenth_gym_ros_humble-*`.
