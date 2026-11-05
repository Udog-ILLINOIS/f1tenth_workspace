# Review of the open SIGRobotics PRs (2026-11-04/05)

Open PRs on `SIGRobotics-UIUC/roboracer`, merged into our fork (`Udog-ILLINOIS/roboracer`) for review,
fixed, built and tested. Nothing is merged into fork `main` or upstream yet: approving and merging
upstream is a human step.

## Branches (fork `Udog-ILLINOIS/roboracer`)

Each = fork main `6f33b94` + SIGRobotics main `1dd15f8` (#20, steering feedback units) + the PRs + review fixes.

| Branch | Contents | Why its own branch |
|---|---|---|
| `review/pr21-map-controller` | PR #21 | control system (MAP controller) and the bridge clock |
| `review/perception-pr22-pr23` | PRs #22 + #23 | same system: localization/mapping evaluation in `perception`, both edit `perception/setup.py` |
| `review/integration` | both, merged, + cross-PR fixes | what the workspace's `planners/sigrobotics` points at on this branch |

Duplicates from a second review pass, safe to delete (this session's git proxy refused branch deletes):
`review/perception-localization`, `claude/roboracer-prs-review-dyhnyq` on the fork.

## Upstream plan (2026-11-05)

#21 and #22 merge cleanly into SIGRobotics main; #23 then conflicts (README.md, ws/scripts/map_track.sh), so it
stays on `review/perception-pr22-pr23` for now. `review/upstream-fixes-pr21-pr22` = SIGRobotics main + #21 + #22 +
their review fixes, with none of our fork-only commits (tests: control 98, perception 117, planning 63, scripts 24
passed; bridge 53 passed + the IPv6-only failure). After #21 and #22 are merged upstream, open the PR
`Udog-ILLINOIS:review/upstream-fixes-pr21-pr22` -> `SIGRobotics-UIUC:main`.

## The PRs

### #21 MAP as an optional racing controller (thegiraffe1)
`controller:=map` in `race.launch.py`: a model-and-acceleration-based controller (L1 guidance, single-track
model, steering lookup table) next to pure pursuit, which stays the default. Also moves the stack onto a
shared `/clock` from the bridge (`use_sim_time:=true` everywhere, `USE_SIM_TIME=false` for wall time),
wheel-slip telemetry and tire-slip analysis tools, a `porto_odom_trial` test track, and a fix for a
CPU-speed-dependent MCL test.

Verdict: **approve with the fixes below; keep MAP opt-in.** The author asks to keep it unmerged until
measured-speed error and high-speed repeatability are resolved: the LUT is provisional and only validated at
speed scale 0.5-0.6. Merging is safe because nothing changes unless `controller:=map` is passed, but the
`/clock` change affects every node.

Fixes: a pose jump or stop while rolling latched MAP stopped for good (now a time-bounded jump check and
`ros2 topic pub --once /map_node/rearm std_msgs/msg/Empty`); heading re-checked against the previous pose
every 40 Hz tick (false faults mid-corner when poses arrive slower than the controller); no stop published
on Ctrl-C; commanded speed could exceed the LUT top and brake mid-straight; encoder jitter read as reversing;
bridge `KeyError` on a frame without the lidar scan rate; `ekf.launch.py` not on `/clock`.

### #22 Localization debugging logger (kaynet2)
`ros2 run perception localization_debug --ros-args -p track:=<name>`: logs any localizer's error against the
sim's ground truth (long/lat/yaw, mean/p95/max), the wheel odometry's distance ratio and the share of time
without a pose. Read-only.

Verdict: **approve.** Small fixes: non-map-frame poses ignored with a warning, a clear message when the track
has no `map_to_world.yaml`.

### #23 SLAM Toolbox mapping backend and diagnostics (rkhanna8)
`map_track.sh <name> --slam-toolbox --wall-follow`: SLAM Toolbox as an opt-in alternative to Cartographer
mapping (Cartographer stays the default), the `slam_graph_tools` C++ graph exporter, mapping diagnostics, plus
race-validation tooling (`validate_race.py`, `analyze_validation.py`) and `AGENTS.md`. Mapping only: SLAM
Toolbox tracks race with `race.sh <name> localizer:=mcl`.

Verdict: **approve with the fixes below.** Needs an image rebuild (`ros-humble-slam-toolbox`).

Fixes: the saved map was compared with itself (always "0 differing pixels"); track dir created 0700
(unreadable from the host); `map_track.sh` refused reruns after an aborted map (dead nodes in `ros2 node list`)
and blocked teleop for manual mapping; refuse SLAM mapping when `/odom` isn't at zero after the reset (the map
origin); diagnostics can no longer abort a good save; `slam_graph_tools` rosdep/CMake fixes.

### Cross-PR (#21 x #23), on `review/integration`
SLAM Toolbox and its pose recorder on the bridge's `/clock`, set by the launch file and not overridden by
the node's yaml; `validate_race.py` ages poses on the stack's clock instead of wall time; `map_node` added to
the competing-controller checks in `map_track.sh` and `validate_race.py`.

## Tests (2026-11-05, ROS 2 Humble image built from the stack's Dockerfile + slam-toolbox)

`colcon build` of all 5 packages (incl. `slam_graph_tools`), then each package's suite in its own pytest
process (`ROS_DOMAIN_ID=42`):

| Suite | base (fork main + #20) | review/integration |
|---|---|---|
| autodrive_bridge | 63 passed, 1 failed* | 71 passed, 1 failed* |
| control | 85 passed | 98 passed |
| perception | 105 passed, 1 failed (MCL burst, CPU timing) | 148 passed |
| planning | 128 passed, 3 failed** | 129 passed, 3 failed** |
| scripts (#21, #23) | n/a | 52 passed |

\* `test_ctrl_c_stops_cleanly`: the bridge binds a dual-stack socket and the test machine has no IPv6
(`Errno 97`); same on every branch. Environment only.
\** `test_obstacle_spliner::test_drives_past_a_long_board_beside_the_wall` (3 cases): fails on fork main, passes
on SIGRobotics main; from our overtaking commit `6f33b94`, not from these PRs (known_errors T19).

Not run: the AutoDRIVE simulator (no live mapping or race laps in this review). The PR authors report live
runs: #21 two recordings at speed scale 0.5/0.6 with no collisions; #23 a two-lap SLAM Toolbox map of
`slam_toolbox_test_v2`. Before merging to fork main, map and race one track live with `controller:=map` and
with `--slam-toolbox`.
