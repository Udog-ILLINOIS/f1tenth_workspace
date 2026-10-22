# Known errors

Errors in the club stack (`planners/sigrobotics`, `roboracer`), the AutoDRIVE sim, and the tooling
around them: what is still open, what was fixed and how it was checked, and what we want changed in
the sim. Add new errors as they are found, and move an entry to **Fixed** when it is fixed.

Severity: high (crashes the car or loses work), medium (wrong results or needs a manual step), low.
IDs: S = club stack review (2026-10-20), W = workspace, T = found while testing, N = found while
testing the fixes (2026-10-21), A = AutoDRIVE sim changes.

## Open

| ID | Where | Error | Impact | Sev. |
|---|---|---|---|---|
| T15 | `localization_gate.py` / Fast DDS | At one race start (1 of 13 gate starts in ws/log) Cartographer's reply to the gate's first `/get_trajectory_states` was dropped ("failed to send response ... client will not receive response", a Humble service discovery race); the gate waited its 10 s `timeout`, retried and localized | First /drive at 16.7 s instead of ~4.5 s; recovers by itself | low, watch: shorter per-call timeout if it recurs |
| N4 | map vs sim world | Cartographer's estimate and the ground truth (mapped through map_to_world.yaml) differ by up to ~1 m in iros2026's second half, the same at the same places in every run, while the car drives cleanly: the map is bent against the sim world (likely from the mapping run's odometry; not checked). So race_monitor.py's localization and raceline errors are only trustworthy near the start, and the gate's ground-truth relocalization can be ~1 m off at a checkpoint far from the start | Wrong test numbers (T5); relocalization error far from the start | medium |
| A6 | sim / `fast_bridge.py` | Resets are detected on the ROS side only: fast_bridge reads them from the sim frames (its own reset command, collision count change, position jump) and publishes `sim_reset`; the sim does not announce them. Without restricted topics (competition) only reset commands are announced, since without ground truth the gate would restart a respawned car at the map origin; the real car has no respawns. In competition a respawn goes undetected and only Cartographer's global search recovers | No relocalization after a crash outside the sim | medium, PARTIAL |
| S5 | `control/control/pure_pursuit_node.py` | Speed taken at the target point (documented in control/README.md); on corner exits it can exceed the profile near `speed_scale` 1.0 | Untested above 0.5 | low, NEEDS DECISION: measure first |
| S1b | `control/control/pure_pursuit_node.py` | No `/scan`-based emergency stop (pose age, frame and offset checks were added) | A wrong pose that passes the checks is not caught | low |
| S3b | `control/control/pure_pursuit_node.py` | No world -> map transform for `/ground_truth/pose` (it is now refused instead of followed) | No ground-truth debug drive | low, NEEDS DECISION |
| S10 | `fast_bridge.py` | IMU held for `imu_reset_hold_frames` (3) frames after a bridge reset. Measured (`bags/iros_new`, bridge without the hold, parked car): after the reset frames' own drop the release reads at most ~3 m/s^2 off gravity, no spike; a reset of a moving car needs them (R7, measured). Respawns, track switches and Scene Reset bypass the bridge's IMU drop: their teleport is announced on `sim_reset`, but its IMU frame is not dropped (decided: with restricted topics the spike only reaches the trajectory the gate discards; without them, as in competition, it reaches the running trajectory) | Maybe one IMU spike into Cartographer | low |
| S32 | `planning/config/racecar.ini` | Tyre B/C/E, mu 0.7, forces are estimates; iros2026 vx sits at the 10 m/s `v_max` cap | Speed profile may not match the sim car | low, needs sim tuning |
| S35 | `autodrive_bridge/config/adapter.yaml` | `speed_kp_brake: 0.1` not tuned | Braking strength unknown above `speed_scale` 0.5 | low, needs sim tuning |
| T2 | `ros2 launch` | SIGTERM to `ros2 launch` alone orphans its nodes (ROS launch behaviour) | Documented in docs/pipeline.md: stop with Ctrl-C or `kill -INT -- -<pgid>` | low |
| T8 / A8 | sim frame rate | App Nap turned off (`NSAppSleepDisabled`, 2026-10-21): the bridge gets 40-55 frames/s with the sim in front or behind, so the background was not the cause. The 94.7 frames/s was a peak in one 30 s log window (the same log had 64, 88, 46.5); the bridge's own work is ~1 ms/frame, so the round trip (sim or Docker port forwarding) limits the rate. A freshly started sim gave 104-112 frames/s (2026-10-21), so the rate falls the longer the sim runs; cause not yet found | Slower control loop than the render rate (~115 fps) | low |
| A4 | AutoDRIVE fork release | `./autodrive.sh fetch` still downloads `f1tenth_multitrack_v1`; only the local app has SimControl | Others don't get the Map button / autoconnect | medium, NEEDS DECISION: publish v2 |
| A9 | `SimControl.cs` | The Map list now starts right of the label, but with a long label ("Map: IROS 2026") the gap is ~90 px instead of 8 (best-fit text overstates its width) | Cosmetic | low |
| W6 | `planners/sigrobotics` | 10 leftover local branches, 3 agent worktrees | Clutter | low, NEEDS DECISION (deletes) |

## Redundancy review (2026-10-21)

Changes from 2026-10-20/21 that duplicate existing infrastructure or each other.

| ID | Duplicate | Existing | Suggestion |
|---|---|---|---|
| R1 | Four nodes detect a sim reset on their own: fast_bridge (reset command), wheel_odom (encoder jump), localization_gate (reset command, collision count, ground-truth jump), corrector (reset guard) | fast_bridge receives every sim frame, including the collision count, first | DONE (2026-10-21): fast_bridge detects (reset command, collision count, position jump), publishes one `sim_reset` and zeroes the command (N2); the gate subscribes to it only. wheel_odom's and corrector's encoder checks stay: they reject an implausible sample of their own input, also for resets the bridge does not see, and an event on another topic has no ordering against the encoder topics (the S7 76 m step) |
| R2 | `ctl reset` set ResetManager's `ResetFlag` directly: it bypassed the bridge, and in Autonomous a Bridge message in the next fixed step (`SocketIOComponent.FixedUpdate`, `V1 Reset` False) could overwrite the flag when ResetManager's Update had already run that frame | `/autodrive/reset_command` (Autonomous with a bridge; also zeroes the command and holds the IMU, S10); `ctl track <current track>` (any mode, no bridge). The sim's Reset button is `Scene Reset`, which reloads the scene | done 2026-10-21 (not committed): `ctl reset` removed; docs give the three ways |
| R3 | `ctl mode` (rewrote the Driving Mode button's private state; wrong if used before the Menu was first opened, since the button's Start() resets to Autonomous) and `ctl menu` | `ctl click <button>` presses those buttons (`Menu`, `"Driving Mode"`, `Map`) | done 2026-10-21 (not committed): both removed, `click` kept |
| R4 | `save_map` finishes the trajectory, polls 5 s for settled poses, then `/write_state` (S12 added the polling) | `cartographer_node -save_state_filename` runs the final optimization and writes the pbstream on shutdown; `cartographer_pbstream_to_ros_map` writes map.pgm/yaml | consider using them; save_map would keep only trajectory.csv and map_to_world.yaml |
| R5 | `world_to_map` written three times (localization_gate, race_monitor.py, bag_analyze.py) | `perception/map_origin.py` owns map_to_world.yaml | one helper in map_origin.py |
| R6 | pure pursuit's pose checks (age, frame, 3 m) and the gate's hold both stop driving on a bad pose | | keep both (the checks also cover the real car); at a sim reset fast_bridge's zero replies act first, the gate's hold keeps the car parked until relocalized |
| R7 | `imu_reset_hold_frames` (S10): 3 extra IMU frames dropped after a reset command | the reset frames' own IMU drop | measured 2026-10-21 (`bag_live_b1`): a reset of a car at 4.1 m/s without the hold reads 60.9, 15.6, 4.8, 1.4, 0.6 m/s^2 off gravity on the teleport frame and the 4 after it; with a reset command the teleport comes 2 frames into the 5 zero frames, so the 3 extra frames cover the tail (with them: max 0.3 m/s^2). Keep |
| R8 | `race_monitor.py` metrics through ground truth | | compare in the map frame (N4) |

## Fixed (2026-10-21/22, committed; in SIGRobotics main through PRs #10-#12)

Each group: a solver and a critic (plan review before code for medium/high, diff review after; low:
one final pass). High (S2/T4/S27/A6) also had two independent verifiers and a live sim test.
Live checks are in the last column.

| ID | Fix | Checked |
|---|---|---|
| S2 | New `perception/localization_gate.py`: starts the localization trajectory with an initial pose relative to the frozen map, relays `/tracked_pose` to `/localized_pose` only once constraints to the map exist, and on a sim reset (fast_bridge's `sim_reset`, R1) closes, finishes the trajectory and restarts it at the ground-truth pose (`map_to_world.yaml`). Retries if Cartographer does not answer. `race.launch.py` drives on `/localized_pose` | berlin (gate's solver): 3 resets and 2 crashes, no crash loop, 0.01-0.02 m after relocalizing; iros2026 (live check): relocalized correctly after every reset but see N1 |
| T4 | Same gate: the car waits until localized (~11-14 s at start and after each reset; 2.4-3.2 s with T4b) | live: settle 11-14 s, then 0.02-0.08 m (before T4b); with T4b see below |
| S27 | `common.sh`: `timeout 5 ros2 topic pub -w 1 ...`, loud message if no bridge | live with the ros2 daemon stopped |
| S1 | Pure pursuit stops on a pose not in `map`, older than 0.5 s, or > 3 m from the raceline (README: what these do and don't catch) | tests (random ROS domain) |
| S3 | Ground-truth debug option removed from docs; such poses are refused (frame check) | tests |
| S4 | Fallback target = farthest point | test |
| S6 | `fast_bridge` sends zeros on SIGINT/SIGTERM; SimControl zeroes the car's command when it drops the connection | live: kill -9 of the bridge, car moved 0.18 m; Ctrl-C, 0.16 m, exit 0.32 s, port freed |
| S7 | Reset jump check on consecutive encoder samples (dt floor 0.02 s) | tests |
| S8 | Gyro dt capped at `max_gyro_dt` 0.1 s | tests |
| S11 | `package.xml` deps; socketio pins explained | review |
| T3 | `wheel_odom` exits cleanly on Ctrl-C (also on the second SIGINT from `ros2 launch`) | live (isolated domain) |
| S12 | `save_map` waits until the optimized poses change and then stay unchanged 5 s before writing | tests |
| S13 | `map.yaml` `free_thresh: 0.196` | tests |
| S14, S15 | `corrector.py` defaults, reset guard, stale encoders | tests |
| S17, S18 | Shutdown exceptions caught, `save_map` exits 130 on Ctrl-C; `tracks_dir` absolute | tests |
| S19-S23 | `first_lap` by start-line crossing; island rule by boundary cells; loop length check; smaller hole fill; clearer optimizer errors | tests; centerlines re-extracted identical on all three tracks |
| S24, S25 | Follow the gap: steering clipped to 0.524 rad; NaN beams never size the bubble (Jerry's code otherwise unchanged) | tests (fail on the old code) |
| S28-S31, S33 | `map_track.sh` needs a terminal, retries `save_map`, notices a dead Cartographer; `.dockerignore`; `follow_the_gap.launch.py` + `ftg` alias | review; `.dockerignore` checked with a throwaway build |
| S34 | Tests for track width sides, first lap, follow the gap callback, NaN scans | 59 tests across 4 packages |
| W2-W4 | Workspace docs: roboracer repo name, two ROS stacks, port 4567 one at a time | review |
| A5 | SimControl keeps the sim's Socket.IO client off while no bridge answers and ends a connection that keeps dropping before its first reply (the sim DLL itself is not patched) | live: 15 s with the bridge down, 940 bytes of log, no leaked sockets |
| T9-T11 | First frame lost, reconnect flood, Docker port forwarding: handled by SimControl | live |
| R2b | `TrackSelector.Switch` (Map button, `ctl track`) set `ResetFlag` the same way as `ctl reset` (R2), so a Bridge message could cancel ResetManager's part of a track switch (encoders, lap and collision counters; the pose is set by the switch itself). It now runs ResetManager's own reset at once (its private Update, by reflection) | live: 11 switches (5 `ctl track`, 6 Map button) with the bridge connected in Autonomous, the car moved before each; encoders back to 0 every time |
| T12 | Test helper `run_race.sh` refuses to start with another stack running | live |
| T13 | Follow the gap (`follow_the_gap_node.py`): ranges are capped at `max_range`, so `argmax` aimed at the first (rightmost) beam of the open road ahead, and the gap search covered the full 270° scan; the car weaved into the walls (81 collisions in ~2 min at 2 m/s on iros2026). Now aims at the middle of the farthest run, gaps only within `fov` ±90° (roboracer 4cb3289) | parked at the iros2026 start: steer -0.231 -> +0.017 rad; live 2026-10-21: 2 m/s, one lap 100.2 s, 0 collisions in 150 s; 2 new tests fail on the old code |
| T14 | `save_map.py`: after `/finish_trajectory` it waited for the poses to change and failed after 30 s if they did not; with the car parked before Enter an earlier optimization already covers every node (final one: "0 computations, 0 constraints"), so the first save of every mapping run failed (the retry worked). Now it warns and takes the poses as final (roboracer 334a129) | live 2026-10-21 (iros2026 as `simtest_iros`): first try failed, retry saved; test changed to expect the poses; 40 perception tests |
| N1 | `adapter.py` (VESC stand-in): the raceline's flying-lap speed (~4.5 m/s) was passed as a speed step at a standstill; the sim's throttle is motor torque, so the wheels spun up in 0.15 s while the car needed 0.8 s, wheel_odom counted ~1.7-2 m of wheelspin, the pose ran ~0.8 m ahead and the car hit iros2026's first corner at every launch. The adapter now ramps the /drive speed like ActuationLimiter in ForzaETH's f1tenth_system: `max_acceleration` 2.5 m/s^2, `max_deceleration` 6.9 m/s^2 (does not limit the planned braking at speed_scale <= 1), restart from max(0, measured speed) after a /drive gap (a respawn can read -10.55 m/s on /odom) | `test_adapter.py` (10 tests, 6 fail on the old code); live 2026-10-21 (`ws/log/bag_live_ab1`): iros2026 3 laps, 0 collisions (was a crash loop); at launch wheel - true speed <= 0.2 m/s, 4 m/s after ~1.45 s, /odom distance error +0.16 m (was 1.7-2 m); corner-exit slip p99 0.2 m/s per lap (lap 1 numbers spoiled by ~1 s bag drops of all sim topics, not slip) |
| N3 | The ~0.8 m forward pose error on the iros2026 start straight was N1's wheelspin, not Cartographer | live: estimate vs ground truth on the start straight max 0.13 m to s ~8 m, 0.21 m to s ~15 m (was ~0.8 m) |
| T5 | `racecar.ini` `width_opt` reverted 0.80 -> 0.50: the 0.22 m error behind the change was N4's skewed metric, and 0.80 did not stop the crash (N1). `test_vehicle_params.py` checks only `width_opt >= width`; `ws/tracks/iros2026_w080` kept as a fallback | live: closest pose centre to a wall over 3 iros2026 laps 0.200 m incl. the launch pass of the first bend (bar >= ~0.19 m), after respawns too (`bag_live_b1`); berlin 0.224 m |
| W5 | PR #10 retitled with a full description, incl. where the AutoDRIVE sim changes live; SIGRobotics main (#11 wall follower, merged 2026-10-22) merged into fork main (c3cb77c), wall follower loads `vehicle.yaml` (ad4acf4) | 86 tests in the container (control 26, perception 19, planning 18, bridge 23); `map_track.sh --wall-follow` after the merge not run live |
| W1 | `planners/sigrobotics` follows roboracer `main` (was pinned at `fd0d5a1`); now at 6019987, identical to SIGRobotics main after PRs #10 (pipeline), #11 (wall follower), #12 (README), all merged 2026-10-22 | `git submodule status`; GitHub compare: identical |
| N2 | After a respawn nothing sent a stop until fast_bridge's 0.5 s `cmd_timeout`, the car rolled ~1.45 m. fast_bridge now detects the reset in the frame (collision count, position jump; restricted topics) or from its reset command, replies zeros for `reset_frames` (5) frames and publishes `sim_reset` before the frame's sensors; the gate subscribes to it only (R1) | `test_fast_bridge.py`; live 2026-10-21 (`ws/log/bag_live_b1`): mid-race reset command at 3.5 m/s and `ctl track` at 4.1 m/s: sim throttle 0 from the first frame after the event, car travel after the teleport 0.002 m (was 1.45 m), no /drive while the gate is closed; `ctl track` gives exactly one event (position jumped) |
| T4b | The gate's 11-14 s wait: `optimize_every_n_nodes` 20 with a node every 0.5 s on a parked car. `localization.lua` `motion_filter.max_time_seconds` 0.04, gate `debounce` 0.5 s | `test_localization_gate.py`; live: start localized 1.6 s after the trajectory started (first /drive 4.2-4.4 s after launch, was 13.9 s); after a mid-race reset 3.2 s and 2.4 s (~2 s of it before the gate starts the new trajectory); no global match during the wait. Still to check: Cartographer CPU on the real car below ~2 m/s |

## Not errors

| ID | Finding |
|---|---|
| S9 | The sim's `V1 Linear Velocity` is already body frame (IMU.cs `InverseTransformDirection`) |
| S16 | AutoDRIVE's vehicle frame is the rear axle centre, like `base_link` |
| S26 | Walking past disparities is Jerry's deliberate tuning (commented-out `break`); left as is |
| T6, T7, A7 | The sim's lap and collision counters are right; the test helper `race_monitor.py` read its start counts before `race.sh`'s reset (also disabling its collision stop). Fixed in the helper: counts increments, a drop = a reset |

## AutoDRIVE sim changes

`autodrive/track_builder` (our `AutoDRIVE` fork, branch `f1tenth_multitrack`): the TrackSelect plugin.

| ID | Change | Status |
|---|---|---|
| A1 | Map row in the Menu panel (after Camera) with a list of tracks; switching restarts the car at the new track's start without restarting the sim | done, tested by clicking through the control port |
| A2 | Connect without clicking (`-autoconnect`, default in `./autodrive.sh sim`), reconnect after bridge restarts | done |
| A3 | Control port 127.0.0.1:4580 (`./autodrive.sh ctl ...`) | done |
| A3b | `click <button>` (default), `screenshot`, `clickat` (layout checks only) | done |
| A4-A9 | see Open / Fixed above | |
