# Repo structure

How the F1Tenth folder maps onto git and GitHub. **Keep this diagram current.** The
`.githooks/check_repo_structure.sh` hook runs on every commit and push. It blocks the commit or push
when a tracked folder (depth 1-2) or a submodule (path, fork, branch) isn't named in this file.
Turn the hook on once per clone: `git config core.hooksPath .githooks`. Claude Code also gets a
reminder from `.claude/settings.json`: before it commits or pushes, and after it pulls, checks out,
switches, merges, rebases or changes submodules.

```
LOCAL  ~/…/SigRobotics/F1Tenth                          GITHUB (Udog-ILLINOIS)                     UPSTREAM (forked from)
═══════════════════════════════                         ══════════════════════                     ══════════════════════
F1Tenth/  ─────────────────────────── git repo ───────► f1tenth_workspace  (main)                  (standalone, not a fork)
│
├── README.md  .gitignore  .gitmodules        ┐
├── .githooks/   (git: structure check)       │
├── .claude/hooks/ + settings.json (Claude)   │
├── sim.sh  autodrive.sh  forzaeth.sh         │
├── docker/                                   │  your files, committed
├── docs/*.md                                 │  directly in f1tenth
├── racelines/{autodrive_roboracer,iros2026}  │
├── data/{raceline_comparison,archive}        ┘
├── docs/F1Tenth_Papers/                      ✗ local only
│
├── autodrive/
│   ├── simulator_multitrack/                 ✗ local only ◄── ./autodrive.sh fetch ── AutoDRIVE  release f1tenth_multitrack_v1
│   ├── track_builder/  ═══ submodule ═══════════════════► AutoDRIVE  (f1tenth_multitrack) ┐
│   └── devkit/         ═══ submodule (shallow) ═════════► AutoDRIVE  (AutoDRIVE-Devkit)   ┴───► Tinker-Twins/AutoDRIVE
│
├── planners/
│   ├── sigrobotics/    ═══ submodule ═══════════════════► roboracer  (main)  club stack, PRs ─► SIGRobotics-UIUC/roboracer
│   │   ├── Dockerfile, docker-compose.yml, ws/src/autodrive_bridge/   (setup + sim bridge; planning etc. empty)
│   │   └── remote f1tenth_old ──► Udog-ILLINOIS/f1tenth  (old club repo, dropped; keeps SimReadyBranch)
│   └── forzaeth/
│       ├── autodrive_forzaeth/               your files, committed
│       ├── race_stack/ ═══ submodule ═══════════════════► ForzaETH  (ros2-humble)  +2 fixes ──► ForzaETH/race_stack
│       └── build_cache/                      ✗ local only
│
├── gym_ros_workspace/
│   ├── f1tenth_gym_ros/ ══ submodule ═══════════════════► f1tenth_gym_ros  (dev-humble)  +edits ► f1tenth/f1tenth_gym_ros
│   └── labs/lab1…lab8                        ✗ local only (each still its own clone of f1tenth/f1tenth_labN_template)
│
└── raceline_optimization/
    ├── README.md  setup.md  autodrive_lines/  f1nn_vs_tum/  archive/   your files, committed (.venv ✗)
    ├── tum_optimizer/  ═══ submodule ═══════════════════► global_racetrajectory_optimization (master) ► TUMFTM/…
    ├── f1nn_init_shehadeh2026/ ═ submodule ═════════════► f1-data-init-optimization  (main) ──► samir-shehadeh/…
    └── track_data/
        ├── full_scale_tracks/    ═ submodule ═══════════► racetrack-database   (master) ──────► TUMFTM/racetrack-database
        ├── f1tenth_scale_tracks/ ═ submodule ═══════════► f1tenth_racetracks   (main) ────────► f1tenth/f1tenth_racetracks
        └── occupancy_maps/       ═ submodule ═══════════► f1tenth_maps         (master) ──────► CPS-TUWien/f1tenth_maps

Legend:  ═══ submodule = f1tenth_workspace stores only a commit pointer to that fork's branch
         ✗ local only  = gitignored (also every venv/.venv, __pycache__, .DS_Store, .vscode/)
```

## How changes flow

`planners/sigrobotics` is the club stack. Only stack work goes there: commit to `main` in
`Udog-ILLINOIS/roboracer`, push, then open a PR to `SIGRobotics-UIUC/roboracer`
(`gh pr create -R SIGRobotics-UIUC/roboracer --head Udog-ILLINOIS:main`).
Everything else (sims, ForzaETH, raceline tools, data) lives in `f1tenth_workspace`.

```
 edit inside a submodule ──► git commit + git push        (goes to the fork, e.g. Udog-ILLINOIS/ForzaETH)
                         └─► cd F1Tenth && git add <path> && git commit && git push   (moves f1tenth_workspace's pointer)

 fork changed on GitHub  ──► git submodule update --remote <path>  ──► commit + push in F1Tenth

 new upstream changes    ──► cd <submodule> && git fetch upstream && git merge upstream/<branch> && git push
```
