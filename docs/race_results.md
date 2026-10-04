# Race results

Timed results from racing each lab's controller on the Humble race stack (`dev-humble`, real
`levine_blocked` map, real `/ego_racecar/lap_count` / `/lap_time` — see [`sim_humble.md`](sim_humble.md)).
Each trial: default parameters, car reset to the start pose `(-12, 0)` via `/initialpose`, 4 laps timed
off `/ego_racecar/lap_time`, collisions checked via `/ego_racecar/collision`.

This is the reference baseline for the roboracer-learn site's per-race metrics — that site isn't
interacted with directly, this log *is* the completed metric for each race.

## Race 1 (reactive controllers: PID wall following, Follow the Gap)

| Lab | Package / node | Steady-state mean (laps 2-4) | Best lap | Collisions | Date |
|---|---|---|---|---|---|
| 3 — PID wall following | `wall_follow` / `wall_follow_node` | **50.42 s** | 50.16 s | 0 | 2026-09-24 |
| 4 — Follow the Gap | `gap_follow` / `reactive_node` | **123.62 s** | 123.54 s | 0 | 2026-09-24 |

**Complete.**

### Lab 3 — PID wall following (`wall_follow_node`)

| Lap | Time (s) |
|---|---|
| 1 | 59.07 (standing start) |
| 2 | 50.47 |
| 3 | 50.63 |
| 4 | 50.16 |

Mean (laps 2-4): **50.42 s**. Best: **50.16 s**. `/ego_racecar/collision` stayed `false` the whole run.

Full lab doc: [`labs/lab3_pid_control/LAUNCH.md`](../gym_ros_workspace/labs/lab3_pid_control/LAUNCH.md).

### Lab 4 — Follow the Gap (`reactive_node`)

| Lap | Time (s) |
|---|---|
| 1 | 147.92 (standing start) |
| 2 | 123.54 |
| 3 | 123.66 |
| 4 | 123.65 |

Mean (laps 2-4): **123.62 s** — about 2.5x lab 3's time on the identical map/loop. Best: **123.54 s**.
`/ego_racecar/collision` stayed `false` the whole run, including through the corner that lab 4's
corner-vs-`levine_obs` bubble-tuning tradeoff (see that lab's doc) is specifically about.

Full lab doc: [`labs/lab4_follow_gap/LAUNCH.md`](../gym_ros_workspace/labs/lab4_follow_gap/LAUNCH.md).

## Race 2 (planning/tracking controllers: pure pursuit, RRT, MPC)

| Lab | Package / node | Steady-state mean (laps 2-4) | Best lap | Collisions | Date |
|---|---|---|---|---|---|
| 5 — Pure pursuit | `pure_pursuit` / `pure_pursuit_node.py` | **55.20 s** | 55.17 s | 0 in laps 1-4, crashed lap 5\* | 2026-09-24 |
| 6 — Motion planning (RRT) | `lab7_pkg` / `rrt_node.py` | **80.27 s** | 79.53 s | 0 in 6 laps | 2026-09-24 |
| 8 — MPC | `mpc` / `mpc_node.py` | **69.73 s** | 69.72 s | 0 in 5 laps | 2026-09-24 |

**Fastest in Race 2: pure pursuit (55.20 s)** — MPC (69.73 s) beats RRT but doesn't catch pure pursuit
here, consistent with its speed cap being tuned for corner safety over raw pace (see its section below).

\* See lab 5's own section below — clean for the 4 timed laps, then crashed into a wall 15.85 s into
the next (untimed) lap. Not a "raced N clean laps" result the way labs 3/4 are.

**Complete** — pure pursuit, RRT, and MPC all done (see [`README.md`](../README.md)).

### Lab 5 — Pure pursuit (`pure_pursuit_node.py`)

Run on Humble (the graded config, `ros2 launch pure_pursuit levine_launch.py`: adaptive lookahead,
lab4-style speed-by-steering-angle, path-curvature anticipatory braking ceiling — see that lab's
`LAUNCH.md` for how the tracker works), tracking `waypoints/centerline_levine_blocked.csv` via
`track:=levine`, same procedure as Race 1 above.

| Lap | Time (s) |
|---|---|
| 1 | 78.99 (standing start) |
| 2 | 55.25 |
| 3 | 55.17 |
| 4 | 55.19 |

Mean (laps 2-4): **55.20 s** — about 1.10x lab 3's PID wall following time on the identical loop
(50.42 s) and well under half of lab 4's Follow the Gap (123.62 s). Best: **55.17 s**.
`/ego_racecar/collision` stayed `false` for all 4 of these timed laps.

**Not stable beyond that, though: the car crashed 15.85 s into the very next (untimed) lap 5.**
Left running past lap 4, `/ego_racecar/collision` flipped `true` and the bridge log shows three
`ego_racecar hit something` warnings within a 450 ms window — `(9.89, 9.19)` → `(10.24, 9.18)` →
`(10.24, 9.18)`, i.e. it hit a wall and got stuck against it rather than grazing past. This is a
**different** corner from the one already documented in `labs/lab5_pure_pursuit/LAUNCH.md`'s "Test
results" (that one's at `x~9.4-9.75, y~-0.15`) — a second sharp point on `levine_blocked` this
tuning doesn't reliably clear. The 4 timed laps above are unaffected (the crash is well after lap 4
completed, not inside the timed window), but this run should **not** be read as "pure pursuit laps
this track cleanly" — it's "pure pursuit clears 4 laps, then fails on the 5th here." Worth
retuning (lower `speed_fast`/`a_lat_max`, or extending `curvature_horizon` to reach this corner)
and re-verifying over more laps before trusting this controller unattended, per the same caveat
`LAUNCH.md` already raises about the `speed_fast=1.5` tuning being "less exhaustively checked."

Full lab doc: [`labs/lab5_pure_pursuit/LAUNCH.md`](../gym_ros_workspace/labs/lab5_pure_pursuit/LAUNCH.md).

### Lab 6 — Motion planning / RRT (`rrt_node.py`)

Run on Humble (`ros2 run lab7_pkg rrt_node.py`, Python side only — see that lab's `LAUNCH.md` for how
the local RRT planner + occupancy grid works, and for the C++ side which is still an unimplemented
skeleton, never run), local RRT re-planned every ~0.2 s around a goal picked off
`waypoints/centerline_levine_blocked.csv`, tracked with simple pure pursuit at a **fixed** `speed=0.8`
m/s (no curvature-regulated speed like lab5's tracker), same reset/timing procedure as above.

| Lap | Time (s) |
|---|---|
| 1 | 103.74 (standing start) |
| 2 | 79.53 |
| 3 | 80.59 |
| 4 | 80.69 |

Mean (laps 2-4): **80.27 s** — about 1.59x lab 3's PID wall following time (50.42 s) and about 1.45x
lab 5's pure pursuit (55.20 s) on the identical loop, which tracks: this is unsmoothed vanilla RRT at a
flat, untuned speed, not a competitively-tuned tracker. Best: **79.53 s**.
`/ego_racecar/collision` stayed `false` for all 4 timed laps.

**Checked past lap 4 for stability, unlike the crash pure pursuit hit here** — left running for laps 5
and 6 (79.15 s, 79.40 s), still zero collisions and no `hit something` warnings in the bridge log
either lap. Not exhaustive (two extra laps, not dozens), but a real data point the other direction from
lab 5's regression: this run got *more* stable past lap 4, not less.

Full lab doc: [`labs/lab6_motion_planning/LAUNCH.md`](../gym_ros_workspace/labs/lab6_motion_planning/LAUNCH.md).

### Lab 8 — MPC (`mpc_node.py`)

Run on Humble (the graded config, `ros2 launch mpc levine_launch.py`, default `PARAMETERS={}`: kinematic
MPC via cvxpy+OSQP, tracking `waypoints/levine_waypoints.csv` — see that lab's `LAUNCH.md` for the model,
tuning, and the 9-bug chain it took to get here), same reset/timing procedure as above. `cvxpy`/`osqp`
aren't persisted in this container (see that lab's "Python dependencies") — reinstalled fresh before this
run, `colcon build --packages-select mpc` confirmed clean.

Lap completions were read directly off `/ego_racecar/lap_count`/`/ego_racecar/lap_time` via a small rclpy
probe (`ros2 topic echo` on these two floods at odom's ~250 Hz republish rate rather than firing only on
lap completion, so raw `echo` output isn't usable for this — the probe only prints on value change).

| Lap | Time (s) |
|---|---|
| 1 | 82.67 (standing start) |
| 2 | 69.72 |
| 3 | 69.73 |
| 4 | 69.73 |

Mean (laps 2-4): **69.73 s** — about 1.38x lab 3's PID wall following time (50.42 s), about 1.26x lab 5's
pure pursuit (55.20 s), but about 0.87x (faster than) lab 6's RRT (80.27 s) on the identical loop. Slower
than pure pursuit/PID by design, not a tuning shortfall — `mpc_config`'s `MAX_SPEED`/the waypoint speed
profile cap this controller's cruise speed at 1.0 m/s (vs. pure pursuit's ~3.33 m/s mean), a deliberate
tradeoff documented in that lab's `LAUNCH.md` for clearing this track's tight corners (min radius 0.741 m)
without wall contact. Best: **69.72 s**. `/ego_racecar/collision` stayed `false` the whole run.

Checked past lap 4 for stability, like lab 6: left running for a 5th lap (69.72 s), still zero collisions.

Full lab doc: [`labs/lab8_mpc/LAUNCH.md`](../gym_ros_workspace/labs/lab8_mpc/LAUNCH.md).

## Not yet raced/timed here

Labs 1, 2 aren't lap-time controllers (ROS basics / emergency braking) so this timed-lap procedure
doesn't apply to them. Every other lab (3-6, 8) now has a Race 1/2 entry above.
