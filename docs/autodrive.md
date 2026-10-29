# AutoDRIVE simulator

AutoDRIVE **RoboRacer** simulator, release
[`2026-iros`](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/tag/2026-iros),
macOS **practice** build, installed 2026-10-04. It replaced Simulator 0.3.0 (2023), which was
deleted. We run our multi-track copy of it: the stock app plus the `TrackSelect` plugin, which adds a
`-track <name>` launch argument (the stock copy was deleted after the multi-track build).

```
autodrive/
  simulator_multitrack/AutoDRIVE Simulator.app   local only: ./autodrive.sh fetch downloads it from
                                                 github.com/Udog-ILLINOIS/AutoDRIVE/releases (f1tenth_multitrack_v3)
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

**Map view** (right-hand HUD, in place of the Front Camera Preview; `MapView.cs`, `MapAccuracy.cs`,
`MapBlocks.cs` in the plugin). The HUD is hidden at some window sizes: `ctl click HUD` toggles it.
- **Ground Truth** tab: the active track's real walls, drawn from its mesh in the style of the stack's
  map, and the car's real pose every frame. The car is drawn to scale (0.55 x 0.31 m) in both tabs.
- **Mapping** tab: the occupancy grid the club stack maps or localizes on (live while mapping) and
  where the stack thinks the car is (TF `map -> base_link`), thresholded as `save_map` writes it. As
  in Ground Truth, the floor is drawn only on the track: the free cells around the stack's pose (no
  pose: the largest free region not reaching the map's edge), wall gaps up to ~4 cells closed for
  that; unknown space, outside the outer wall and inside islands stay dark (map_server marks those
  free). It
  comes from the club stack's `autodrive_bridge map_view` node (started by `autodrive_launch.py`) over
  TCP to 127.0.0.1:4581 (`host.docker.internal` from the container). Once the stack sends poses, the
  map is placed where its walls best fit the real ones and drawn in the Ground Truth frame and scale,
  so switching tabs leaves the track in place. Without poses (no stack running, only its last map) the
  map is placed by its walls alone (turns every 5 deg, refined), so it has the Ground Truth
  orientation then too; only when no turn clearly fits best is it centred, with "not aligned yet".
  It clears on a track switch until the stack sends its map again.
- Under the figure: **Map error** (mean distance of the map's walls from the real wall surface; a
  perfect 5 cm map scores ~1.3 cm on Porto) and **Localization error** (mean distance of the
  estimated `base_link` from the real one at the time of each estimate, last 5 s). On `porto_v2`:
  ~2.2 cm and ~2.5-3.5 cm.
- **Block bar** (Ground Truth tab only), the thin strip above the figure: drag its dark grey square
  onto the track to put a 0.3 x 0.3 x 0.4 m block on the real track there (the lidar sees it, the car
  collides with it); drag a block on the figure to move it, or back into the strip to remove it.
  The control port does the same with `ctl obstacle <x> <y> [size]` (ROS world frame) and
  `ctl obstacle clear`; `ctl drag <x0> <y0> <x1> <y1>` replays a mouse drag (positions as `clickat`,
  for tests). Blocks are gone after a Scene Reset.
- Tabs from the control port: `ctl click "Ground Truth Tab"`, `ctl click "Mapping Tab"`.

**Gamepad** (`Gamepad.cs` in the plugin). Drives the car in **Manual** mode only. Autonomous mode is
always the bridge, and Manual without a pad is the stock keyboard (W/S, A/D). macOS doesn't see
XInput pads (Logitech F710/F310 in X mode, Xbox 360 pads), so the plugin reads them over USB with
libusb (`brew install libusb`; without it the gamepad is off). The pad is opened by itself whenever it
is plugged in, with a short buzz and LED 1, and `ctl status` shows it as `"gamepad"`. The keyboard
still works while a pad is open.
- RT throttle (proportional to how far it is pulled, times the power level), LT brake then reverse
  once stopped, left stick steering, B handbrake, Start =
  car back to the start pose, LB/RB power level (25-100 %, starts at 55 %), Y steering assist on/off,
  X traction control on/off (off by default). Changes show at the bottom of the window.
- Handling: stick deadzone and response curve, steering eased in and out, steering assist (full stick
  scales down with speed to about the grip limit, 12 m/s^2), traction control when on (throttle ramps
  and is capped at low speed, since the sim throttle is wheel torque), and coasting with light engine braking
  off the pedals instead of the stock full brake at zero throttle. The tuning constants are at the top
  of `Gamepad.cs`.
- While the pad drives, VehicleController is disabled and its own manual step (Steer, Drive, wheel
  poses) runs from the plugin with the pad's inputs, so the car physics are the stock ones.

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
