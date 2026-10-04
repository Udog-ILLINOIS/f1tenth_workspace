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

The sim sends one sensor frame per rendered frame, so the small low-quality window is what
gives a fast control loop (32–45 Hz, against about 12 Hz fullscreen at top quality on the
old build).

In the sim, connect to `127.0.0.1:4567` and switch to autonomous mode. The ROS 2 side lives
in the ForzaETH container: see [`forzaeth_autodrive.md`](forzaeth_autodrive.md). It uses
its own bridge (`planners/forzaeth/autodrive_forzaeth`, `fast_bridge`), with the same topics as the
devkit's `autodrive_roboracer` bridge:
`/autodrive/roboracer_1/{ips,imu,odom,lidar,throttle,steering,*_encoder,lap_count,lap_time,last_lap_time,best_lap_time,collision_count}`,
commands on `.../throttle_command` and `.../steering_command`, and `/autodrive/reset_command`
(Bool) to put the car back on its spawn.

Practice, explore and compete builds exist for macOS, Linux and Windows. Explore and compete
weren't downloaded. The devkit's `autodrive_roboracer` package ships in the release zip; the
`AutoDRIVE-Devkit` branch has the F1TENTH-named equivalent (`ADSS Toolkit/autodrive_ros2/autodrive_f1tenth`).
If macOS refuses to open the app: `chmod -R +x "AutoDRIVE Simulator.app/Contents/MacOS"` and
`xattr -cr "AutoDRIVE Simulator.app"` (already done for this install).
