# AutoDRIVE simulator

AutoDRIVE **RoboRacer** simulator, release
[`2026-iros`](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/tag/2026-iros),
macOS **practice** build, installed 2026-10-04. It replaced Simulator 0.3.0 (2023), which was
deleted. We run our multi-track copy of it: the stock app plus the `TrackSelect` plugin, which adds a
`-track <name>` launch argument (the stock copy was deleted after the multi-track build).

```
autodrive/
  simulator_multitrack/AutoDRIVE Simulator.app   local only: ./autodrive.sh fetch downloads it from
                                                 github.com/Udog-ILLINOIS/AutoDRIVE/releases (f1tenth_multitrack_v1)
  track_builder/   submodule, Udog-ILLINOIS/AutoDRIVE branch f1tenth_multitrack: plugin source,
                   custom tracks, build_track.py (rebuilding needs the stock 2026-iros app, see its README)
  devkit/          submodule, Udog-ILLINOIS/AutoDRIVE branch AutoDRIVE-Devkit (reference only)
autodrive.sh       launcher (repo root)
```

```bash
./autodrive.sh fetch        # once, after cloning
./autodrive.sh sim          # 1280x720 window, "Very Low" quality
./autodrive.sh sim --full   # the app's own resolution and quality
```

`./autodrive.sh sim` connects to the bridge on 127.0.0.1:4567 by itself (`-autoconnect`; `--no-connect`
to skip) and stays connected across bridge restarts, so Connect never has to be clicked. Driving
Mode already starts as Autonomous. The sim is also driven from its control port, 127.0.0.1:4580
(the `SimControl` part of the TrackSelect plugin, `autodrive/track_builder/track_select/SimControl.cs`):

```bash
./autodrive.sh ctl status            # connected?, frames/s, last frame age, track, driving mode
./autodrive.sh ctl tracks            # track names for `track`
./autodrive.sh ctl track porto       # switch track, car at its start pose (no sim restart)
./autodrive.sh ctl connect | disconnect | reconnect
./autodrive.sh ctl click Menu        # press a sim button by name: Menu, "Driving Mode", Map, "Map Porto", "Scene Reset", ...
./autodrive.sh ctl screenshot /tmp/sim.png   # the sim's own picture only, no other windows
```

`click <button>` is the way to press the sim's buttons: it sends the button the pointer events of a
mouse click, so the sim's own handler runs (Connect, Driving Mode, Scene Reset, the Map list, ...),
as long as the button is visible (open the Menu first). It does not move the real mouse.
`ctl tree <object>` lists button names. `ctl clickat <x> <y>` clicks whatever is drawn at a position
(0-1 of the window, from its top left); positions change with the window size and display scaling,
so use it only to check the layout.

- Driving mode: `ctl tree "User Interface" 1` shows `Menu ... (inactive)` when the Menu is closed;
  if it is, `ctl click Menu` (it toggles); then `ctl click "Driving Mode"` (toggles Autonomous /
  Manual), and `ctl status` shows the mode.
- Car back to its start pose: in Autonomous with a bridge, `/autodrive/reset_command`
  (`./forzaeth.sh reset`, or `reset_sim_car` in the club stack's `ws/scripts/common.sh`), which also
  zeroes the command (the club bridge also holds the IMU, S10). Otherwise (Manual, no bridge):
  `ctl track <current track>` (or its Map row), which the bridge does not see. `Scene Reset` reloads the whole scene (the sim
  reconnects by itself only when started with `-autoconnect` or after `ctl connect`).

The Menu panel (top-left button) has a **Map** row after Camera: it lists the tracks, and picking
one switches to it the same way (car at the new track's start pose, which is also where
`/autodrive/reset_command` and `ctl track` on the current track put it). Known sim problems:
[`known_errors.md`](known_errors.md).

The sim sends one sensor frame per rendered frame, so the small low-quality window is what
gives a fast control loop: about 104-112 frames/s from a freshly started sim, falling to
~40-55 after hours of running (cause not found, `known_errors.md` T8), against about 12 Hz
fullscreen at top quality on the old build.

The sim connects to whichever bridge is on `127.0.0.1:4567`. There are two ROS 2 stacks, each with
its own container and bridge:

- Club stack: `cd planners/sigrobotics && docker compose up -d && docker compose exec ros bash`, then
  `ros2 launch autodrive_bridge autodrive_launch.py` (see `planners/sigrobotics/README.md`).
- ForzaETH: `./forzaeth.sh up && ./forzaeth.sh bridge` (`planners/forzaeth/autodrive_forzaeth`,
  `fast_bridge`); see [`forzaeth_autodrive.md`](forzaeth_autodrive.md).

Both containers publish host port 4567, so only one can run at a time: stop the other first with
`./forzaeth.sh down` or `cd planners/sigrobotics && docker compose down`. Both bridges use the
devkit's `autodrive_roboracer` topics:
`/autodrive/roboracer_1/{ips,imu,odom,lidar,throttle,steering,*_encoder,lap_count,lap_time,last_lap_time,best_lap_time,collision_count}`,
commands on `.../throttle_command` and `.../steering_command`, and `/autodrive/reset_command`
(Bool) to put the car back on its spawn.

Practice, explore and compete builds exist for macOS, Linux and Windows. Explore and compete
weren't downloaded. The devkit's `autodrive_roboracer` package ships in the release zip; the
`AutoDRIVE-Devkit` branch has the F1TENTH-named equivalent (`ADSS Toolkit/autodrive_ros2/autodrive_f1tenth`).
If macOS refuses to open the app: `chmod -R +x "AutoDRIVE Simulator.app/Contents/MacOS"` and
`xattr -cr "AutoDRIVE Simulator.app"` (already done for this install).
