# AutoDRIVE simulator

AutoDRIVE **RoboRacer** simulator, release
[`2026-iros`](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/tag/2026-iros),
macOS **practice** build, installed 2026-10-04. It replaced Simulator 0.3.0 (2023), which was
deleted.

```
autodrive/
  simulator/AutoDRIVE Simulator.app   universal binary (native arm64)
  simulator/README.md                 upstream macOS notes (chmod / quarantine)
  devkit/                             upstream ROS 2 devkit, package autodrive_roboracer (reference only)
autodrive.sh                          launcher (repo root)
```

```bash
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
weren't downloaded. Simulator and devkit come from the same release; upgrade them together.
If macOS refuses to open the app: `chmod -R +x "AutoDRIVE Simulator.app/Contents/MacOS"` and
`xattr -cr "AutoDRIVE Simulator.app"` (already done for this install).
