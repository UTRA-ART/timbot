# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

## 1. What this is

**Timbot** is the autonomous ground rover built by **UTRA-ART** (University of Toronto Robotics Association – Autonomous Rover Team) for the **IGVC** (Intelligent Ground Vehicle Competition). It is a ROS 2 **Humble** workspace (Ubuntu 22.04, Python 3.10) containing the full autonomy stack: sensor drivers, localization/SLAM, computer vision (lane + obstacle + ramp detection), Nav2 path planning, GPS waypoint following, and low-level motor control.

It runs in two modes from a **single launch entry point**:
- **Simulation** (Ignition Gazebo) on a dev laptop — `config:=sim.yaml`
- **Competition / real rover** (physical sensors over USB, motors over a Raspberry Pi) — `config:=comp.yaml`

The repo is a "monorepo of ROS packages" grouped into functional directories (`cv/`, `nav/`, `odom/`, `motor/`, `sensor_drivers/`, `embedded/`, `description/`, `launch/`, `misc/`). Each directory holds one or more independent colcon packages. Vendor drivers are git **submodules** under `sensor_drivers/` and `misc/`.

---

## 2. Build & run

### Build
The standard command is plain colcon from the repo root:
```bash
colcon build --symlink-install        # --symlink-install matters for the Python-via-CMake packages
source install/setup.bash             # required in every new shell
```
A `build` shell alias is used on the dev/rover machines (see `commands.txt`). **Deactivate conda before building** (`conda deactivate`) — the CMake packages auto-detect `$CONDA_PREFIX` and a stale env causes Python-library mismatches. ROS Humble expects the system Python 3.10.

`scripts/build.sh` is the **full first-time / CI build**. It does extra work that plain `colcon build` does **not**:
1. Resets the `sensor_drivers/navsat` submodule and applies `patches/navsat_fix.patch`.
2. Builds `sensor_drivers/zed_open_source` (the Stereolabs zed-open-capture C++ lib) via raw CMake/make — this is **not** a colcon package.
3. Runs `colcon build`.

Build a single package:
```bash
colcon build --symlink-install --packages-select motor_control
```

### `COLCON_IGNORE` — important gotcha
There are **local, untracked** `COLCON_IGNORE` files in `sensor_drivers/`, `misc/`, and `log/` (confirm with `git ls-files | grep COLCON_IGNORE` → none tracked). On the current dev machine these **exclude the submodule drivers and `misc/` from the build**, so `install/` here contains only the ~12 first-party packages (`description`, `odom_state`, `lane_detection`, `depth_detection`, `nav_stack`, `load_waypoints`, `filter_lidar_data`, `motor_control`, `motor_odom`, `ros_scripts`, `timbot_gui`, `gazebo_worlds`, `timbot_launch`).

Consequences to remember:
- A fresh clone will **not** have these ignore files, so a first build attempts every submodule (hence `scripts/build.sh` + `git submodule update --init --recursive`).
- On a machine where `misc/` is ignored, **`twist_mux` is missing** even though `robot_bringup.launch.py` includes it in both sim and real modes — launch will fail until `twist_mux` is built. Same for `phidgets_spatial`, `rplidar_ros`, `nmea_navsat_driver` referenced by `comp.yaml`.
- If a package referenced by a launch file "isn't found," check for a stray `COLCON_IGNORE` before anything else.

### Submodules
```bash
git submodule update --init --recursive
```
Submodules (see `.gitmodules`): `sensor_drivers/imu` (phidgets_drivers), `sensor_drivers/rplidar` (Slamtec rplidar_ros), `sensor_drivers/navsat` (nmea_navsat_driver — **patched**), `sensor_drivers/zed_open_source` (Stereolabs zed-open-capture), `misc/twist_mux` (fork), `misc/ds4_driver` (PS4 controller). `sensor_drivers/zed_camera/` is the official ZED ROS 2 wrapper (currently untracked working copy, not a submodule).

### Run
```bash
# Simulation (Gazebo + full stack)
ros2 launch timbot_launch timbot.launch.py config:=sim.yaml

# Competition / real rover (all sensors on the USB hub)
ros2 launch timbot_launch timbot.launch.py config:=comp.yaml
```
On the **team laptop** the launch auto-sets NVIDIA offload + `QT_QPA_PLATFORM=xcb` (driven by `team_laptop: true` in the config). If RViz/Gazebo crash, the sim.yaml header documents the manual env-var workarounds.

### Tests
`colcon test` / `colcon test --packages-select <pkg>` / `colcon test-result --all`. Note: this is a competition codebase — there is **little real unit-test coverage**; most validation is done by running sim and on-vehicle. Don't assume green tests mean working behavior.

### Python dependencies
`pip install -r requirements.txt` (numpy, opencv, torch/torchvision/ultralytics for YOLO lane detection, scipy, `utm`/`pyutm` for GPS conversion, `empy==3.3.4` + `lark` for ROS message generation).

---

## 3. The big idea: config-driven launch orchestration

**Everything starts from `launch/launch/timbot.launch.py` + a YAML in `launch/config/`.** Read these two files first — they encode the entire system topology. This is the single most important architectural concept in the repo.

`timbot.launch.py` is a generic **sequential orchestrator**. It does not hard-code the pipeline; it reads the chosen YAML and:

1. **Pipeline stages** are defined in `LAUNCH_STAGES` (an ordered list): `Gazebo → Spawn → Robot Bringup → Odom State → Filter Lidar → Lane Detection → Depth Detection → Cartographer → RViz → Nav Stack → Load Waypoints`. Each stage is launched **only if** `<stage>.enabled: true` in the YAML.
2. **Sequencing between stages** uses one of two modes (global `use_topic_check`):
   - **Topic-check mode (default, `use_topic_check: true`):** after launching a stage, an `ExecuteProcess` shell waiter polls `ros2 topic list` until that stage's `expected_topics` all appear, then fires the next stage via an `OnProcessExit` event handler. This is event-driven readiness gating built out of shell loops — see `_make_topic_waiter()` and `build_stage_chain()`.
   - **Timer mode (`use_topic_check: false`):** falls back to a blind `TimerAction(period=delay_sec)` between stages.
3. **Hardware drivers (real rover only, `sim: false`):** `build_hardware_driver_stages()` builds a separate driver chain that runs **before** the main pipeline: `refresh_motors → IMU → LiDAR lower → ZED open-capture → RPi sync (wheel_odom)`. (GPS and upper-LiDAR stages are currently commented out.) Each driver stage waits for its own expected topic (e.g. `/imu/data`, `/scan_lower`, `/zed_node/left/image`, `/wheel_odom`) before the next. In sim these are skipped entirely — Gazebo plugins publish the equivalent topics.
4. Per-environment hardware settings (serial ports, ZED video device, baud rates, camera exposure/disparity params) come from the **top of the YAML**, so this file rarely needs editing — change `comp.yaml` instead.
5. It forces `/usr/bin` to the front of `PATH` so `#!/usr/bin/env python3` nodes resolve to system Python, not a conda env.

**To change what runs or in what order, edit the YAML config, not the launch file.** Adding a new subsystem = add a `launch_*` stage function + an entry in `LAUNCH_STAGES` + a config block with `enabled`/`expected_topics`/`delay_sec`.

### Key differences between `sim.yaml` and `comp.yaml`
| | sim.yaml | comp.yaml |
|---|---|---|
| `sim` | `true` | `false` |
| `gazebo` / `spawn` | enabled | disabled (real sensors instead) |
| nav config | `nav2_rpp.yaml` (Regulated Pure Pursuit) | `espresso_nav2_bkup.yaml` |
| `load_waypoints` | enabled | disabled |
| lane camera res | 640×320 | 672×376 (ZED native) |
| `datum` / magnetic declination | zeros | real GPS datum + declination for the test site |
| hardware ports | n/a | `/dev/gps_port_0`, `/dev/lidar_port_0`, `/dev/video2` |
| `ramp_seg_using_lidar` | `true` | `false` |

---

## 4. Localization & TF architecture (the second hard-to-grok part)

This is a **dual-EKF + Cartographer + navsat** stack from `robot_localization`, configured in `odom/odom_state/config/odom.yaml` and launched by `odom/odom_state/launch/odom_state.launch.py`. SLAM is Cartographer, configured in `description/config/cartographer.lua`. Understanding the TF frame ownership is essential — multiple nodes could publish `map→odom`, and the design carefully assigns exactly one owner.

**TF tree & data flow:**
```
utm ──(navsat_transform_node)──► map ──(ekf_global)──► odom ──(ekf_local)──► base_link ──► {sensors}
```

- **`ekf_local`** (`robot_localization/ekf_node`): fuses `/wheel_odom` (full pose+vel) + `/imu/data` (orientation + angular vel + linear accel). Publishes `odom→base_link` TF and `/odometry/local`. `world_frame=odom`, `publish_tf=true`, 30 Hz, 3D (`two_d_mode: false`).
- **Cartographer** (`cartographer_ros`): local SLAM only. Inputs (remapped in `launch_cartographer`): `scan=/scan_modified`, `points2_1=/zed_node/left/obstacle_points`, `points2_2=/cv/lane_detections_cloud`, `odom=/odometry/local`, `imu=/imu/data`, `fix=/gps/fix_cov`. `tracking_frame=imu_link`, `published_frame=odom`, `use_nav_sat=false`, `num_point_clouds=2`. Publishes `/tracked_pose` and `/map` (via `cartographer_occupancy_grid_node`). It does **not** use GPS — purely lidar+odom+IMU local SLAM.
- **`pose_relay.py`**: wraps Cartographer's `/tracked_pose` (PoseStamped) into `/tracked_pose_cov` (PoseWithCovarianceStamped) with configurable covariance, so the global EKF can consume it.
- **`gps_cov_relay.py`**: stamps covariance (from `horizontal_stddev`/`vertical_stddev`) onto the raw GPS `NavSatFix` → `/gps/fix_cov`.
- **`navsat_transform_node`**: converts GPS lat/lon ↔ UTM and owns the `utm→map` TF. Consumes `/gps/fix_cov`, `/imu/data`, `/odometry/global`; emits `/odometry/gps` and `/gps/filtered`. Configured via `datum` + `magnetic_declination_radians` + `yaw_offset: π/2` (ENU correction).
- **`ekf_global`** (`robot_localization/ekf_node`): fuses Cartographer pose (`/tracked_pose_cov`, absolute) + GPS (`/odometry/gps`, the global anchor). `world_frame=map`, **`two_d_mode: true`**, 10 Hz. **It is the sole authority on `map→odom`** but `publish_tf: false` here — read the in-file comments carefully; the intended owner of `map→odom` and GPS-vs-SLAM "two absolute inputs" conflict resolution is a known sharp edge that has been re-tuned repeatedly (see git log).

When editing odom: the **process_noise_covariance** matrices and the per-sensor `*_config` boolean masks `[x,y,z,roll,pitch,yaw, ẋ,ẏ,ż,roll˙,pitch˙,yaw˙, ẍ,ÿ,z̈]` are the main tuning knobs. The git history shows this stack is fragile and heavily tuned; prefer small, documented changes.

---

## 5. Module-by-module

### `description/` — robot model, bringup, SLAM/RViz config
- `rover_model/urdf/timbot.urdf.xacro` (+ `stand_link`, `wheels`, `constants.xacro`, `plugins.ignition.xacro`): differential-drive rover, SolidWorks-exported. **Frames live here**: `base_link`, `chassis_link`, `stand_link`, `top_lidar_link`/`bottom_lidar_link`, `zed_link`→`left_camera_link`→`left_camera_link_optical` (optical frame is REP-103 rotated), `imu_link`, `gps_link`. `constants.xacro` holds all dimensions (wheel diameter 0.25 m, wheel separation 0.823976 m, ZED pitch 20°, lidar params, ref lat/lon for U of T). Note: `imu_link` is deliberately placed at `base_link` height (Cartographer requirement) while the visual mesh is offset up to the real stand position.
- `launch/robot_bringup.launch.py`: Robot State Publisher (URDF→`/robot_description`,`/tf`) + Joint State Publisher (real only; Gazebo provides joint states in sim) + **twist_mux** + pointcloud relay. Runs in **both** sim and real, **before** spawn (sim) / odom (real). The relay node (`pointcloud_frame_relay.py`) republishes `/zed_node/left/points` as `/zed_node/left/points_rviz` with the correct frame id (`left_camera_link` in sim, `left_camera_link_optical` on real) — this is the input feed for `depth_detection`.
- `launch/spawn.launch.py`: **sim-only** ROS↔Gazebo `parameter_bridge` (clock, joint_states, `/scan_lower`,`/scan_upper`, `/gps/fix`, `/imu/data`, `/cmd_vel`, set/delete/create entity services, `/odom`) + camera bridge (gated on `enable_camera`) + `ros_gz_sim create` to spawn the robot from `/robot_description`.
- `config/cartographer.lua`, `rviz/timbot.rviz`, `scripts/pointcloud_frame_relay.py` (re-stamps pointcloud frame_id for RViz; `left_camera_link` in sim vs `..._optical` on real).

### `odom/odom_state/` — see §4. Executables: `pose_relay.py`, `gps_cov_relay.py` (installed via CMake `install(PROGRAMS ... RENAME)`).

### `cv/` — computer vision
- `lane_detection/`: `lane_detection_inference.py` subscribes synced RGB (`/zed_node/left/image`, remapped from `image`) + depth (`/zed_node/left/depth_image`) via `message_filters.ApproximateTimeSynchronizer`, plus `/zed_node/left/camera_info` for intrinsics. Two backends chosen by `lane_detection_mode` param: **`0` = deep learning** (YOLO via `ml_lane_detection.py`, model `models/best_model_int8.pt`, needs torch/ultralytics) and **`1` = classical** (`classical_lane_detection.py`, HSV white-threshold + morphology — the comp default). Outputs: `/cv/model_output` (debug image) and **`/cv/lane_detections_cloud`** (PointCloud2 of lane pixels back-projected through depth, in `left_camera_link_optical`) → fed to Cartographer + costmaps. `roi_mask_points` (normalized polygon) masks out the rover's own front.
- `depth_detection/`: `pointcloud_filter_from_rviz.py` (node `pointcloud_rviz_filter`) filters the already-computed ZED point cloud (`/zed_node/left/points_rviz`). **Depends on `lane_detection` being enabled** — `pointcloud_relay` in `robot_bringup` only publishes `/zed_node/left/points_rviz` when `lane_detection.enabled: true`, so `timbot.launch.py` automatically skips this stage if lane detection is off. — the ZED driver (`zed_open_capture_node`) already does the disparity→depth computation, so this node only applies ROI/height filtering, voxel downsampling, BFS clustering, and elevation-grid ramp classification. Outputs: `/zed_node/left/depth_points` (ROI-filtered), **`/zed_node/left/obstacle_points`** (clustered obstacles → Cartographer/costmaps), `/zed_node/left/ramp_points` (ramp PointCloud2). When `ramp_seg_using_lidar: false` in the active YAML, also publishes detected ramps as `PoseArray` to `/ramp_seg` (the topic `ramp_navigate.py` consumes). Parameterised via `config/depth_detection.yaml` with live `parameters_callback` reconfiguration.

### `nav/` — navigation
- `nav_stack/`: Nav2. `launch/move_base.launch.py` brings up `planner_server`, `controller_server`, `behavior_server`, `bt_navigator`, and a `lifecycle_manager` (autostart). The active params YAML is selected by the **launch config** (`nav2_rpp.yaml` in sim, `espresso_nav2_bkup.yaml` in comp). `nav2_rpp.yaml` uses **SmacPlannerHybrid** (global) + **RegulatedPurePursuitController** (local), 10 Hz controller. The `config/` dir has many experimental variants (`espresso_*`, `nav2_smac`, `nav2_one`, `*_old`, custom BT XMLs) — most are dead/experimental; only the one named in the YAML is live.
- `load_waypoints/`: GPS waypoint mission executor. `navigate_waypoints.py` (node `load_waypoints_server`) loads a JSON from `jsons/`, converts lat/lon→UTM→`map` frame via TF, and drives the `NavigateToPose` action sequentially in a daemon thread (MultiThreadedExecutor). Handles laps, clockwise/CCW ordering, optional auto-generated corner waypoints, **respawn** (Gazebo teleport + EKF `set_pose` + Cartographer trajectory restart, via `RoverRespawn.srv`), and **ramp interrupts**. Stops the autonomous loop the moment a manual `/goal_pose` arrives from RViz. `ramp_navigate.py` is a state machine (`no_ramp→to_ramp→on_ramp`) that takes over to cross ramps detected on `/ramp_seg` and pauses waypoints via `/ramp_naving`. `nav_options.py` exposes the `/rover_navigation` service for manual abs/rel/gps goals (`RoverNavigation.srv`). JSON waypoint files live in `jsons/` (`competition_points`, `IGVC_course`, `outside*`, `sim_waypoints_fixed`).
- `filter_lidar_data/`: `dual_lidar_filter_node.py` compares lower/upper LiDARs to detect & remove ramp points (geometry: `depth = lidar_distance / tan(theta)`), outputs cleaned **`/scan_modified`** (the scan Cartographer/Nav2 actually use). Only publishes `/ramp_seg` (`PoseArray`) when `ramp_seg_using_lidar: true` in the active YAML — otherwise the camera-based `depth_detection` node owns that topic. In current comp.yaml both `main_lidar_topic` and `upper_lidar_topic` point at `/scan_lower` (single physical lidar), so it mostly limits the output FOV to 180°.

### `motor/` — drive & wheel odometry (run on the Raspberry Pi)
- `motor_control/src/motor_control.py` (node `motor_control_node`): subscribes `/cmd_vel` (output of twist_mux) + `pause_navigation`, converts (v, ω) → per-wheel duty cycle via measured linear fits (`convert_speed_left/right`), drives **RPi GPIO PWM** pins, reads encoder ticks back over **serial from an Arduino** (`/dev/ttyACM0`, 115200, packet format `<left,right>`), and publishes `/left|right_wheel/ticks` (Int32) + `/left|right_wheel/direction` (Bool). 30 Hz read / 15 Hz control. Drives a status light (solid=manual, blink=autonomous) and a **TIMEOUT safety** that zeros the motors if no `/cmd_vel` for `TIMEOUT` and not in autonomous mode. **Imports `RPi.GPIO` and `serial`** — only runnable on the Pi.
- `motor_odom/src/odom_pub.py` (node `wheel_odom_pub`): integrates the tick/direction topics into **`/wheel_odom`** (nav_msgs/Odometry, `odom→base_link`) at 30 Hz — the dead-reckoning input to `ekf_local`. Differential-drive kinematics; `WHEEL_RADIUS=0.1375`, `WHEEL_BASE=0.69`. Supports pose set/reset topics.
- Deployment: the Pi runs `motor_control.service` (systemd). `scripts/start_motor_control.sh` waits for the `10.42.x` link-local IP from the laptop, sources the workspace, and launches it. SSH alias `utrapi`. Check with `systemctl status motor_control.service`.

### `embedded/` — microcontroller firmware & glue
- `arduino/read_hall/read_hall.ino`: Arduino sketch reading wheel hall-effect encoders, emits `<left,right>` packets.
- `fault_monitor/`: Arduino fault-monitor sketch + `Logger.h`.
- `ros_scripts/read_odom_arduino.py` (node `ticks_publisher`): alternative serial→ticks bridge (uses `/left|right_wheel/command` to infer direction). Overlaps with `motor_control.py`'s built-in serial reader — know which one is active in a given setup.

### `sensor_drivers/` — vendor drivers (mostly submodules; often `COLCON_IGNORE`d locally)
- `imu/` = phidgets_drivers (use `phidgets_spatial` `spatial-launch.py`; raw `/imu/data_raw` remapped to `/imu/data`).
- `rplidar/` = Slamtec rplidar_ros (`rplidar_a1_launch.py`; `/scan`→`/scan_lower`).
- `navsat/` = nmea_navsat_driver, **patched** by `patches/navsat_fix.patch` (`/fix`→`/gps/fix`). Currently the GPS driver stage is commented out in `timbot.launch.py`.
- `zed_open_source/` = Stereolabs zed-open-capture C++ lib (built by `scripts/build.sh`, not colcon).
- `zed_camera/` = official ZED ROS 2 wrapper (`zed_wrapper`, `zed_components`, `zed_ros2`, `zed_debug`) — untracked working copy.
- `zed_camera_depth_cloud/` = original home of `pointcloud_filter_from_rviz.py`; its content has been moved to `cv/depth_detection/`. This directory is now redundant and can be deleted.
- The custom **`timbot_launch` C++ node** `zed_open_capture_node` (`launch/src/zed_open_capture_node.cpp`, launched by `launch/launch/zed_open_capture.launch.py`) is the real-rover camera driver: it reads the raw stereo USB feed via zed-open-capture, runs StereoSGBM disparity→depth on CPU, and publishes the `/zed_node/left/{image,camera_info,depth_image,points}` topics that the CV nodes expect — i.e. it reproduces, on real hardware, the topic contract the Gazebo camera bridge provides in sim.

### `misc/` — teleop & cmd_vel muxing (often `COLCON_IGNORE`d locally)
- `twist_mux/` (forked): priority mux feeding the final `/cmd_vel`. Priorities (`config/twist_mux_topics.yaml`): `joy_vel`=100, `key_vel`=90, `nav_vel`=10 — **manual teleop always overrides Nav2**. Locks: `pause_navigation`, `joy_priority`, `stop_closing_loop`.
- `ds4_driver/` PS4 controller, `teleop_twist_joy/` (`ds4_config.yaml`), `teleop_twist_keyboard/`.
- `debug/cmd_vel_subscriber.py` debug helper.

### `gazebo_worlds/` — `worlds/` (`track.world` default, plus `ramp_track`, `track_no_walls`, `empty`) + `launch/gazebo.launch.py` (runs `ign gazebo`, sets `IGN_GAZEBO_RESOURCE_PATH`, `LIBGL_ALWAYS_SOFTWARE` from `software_rendering`).

### `timbot_gui/` — PyQt5 dashboard (`ros2 run timbot_gui timbot_gui`): node/topic monitor, one-click subsystem launch (sim/comp selector), and a `/cmd_vel` rosbag/CSV recorder ("BagWriter") using `rosbag2_py`. The **only `ament_python` package** in the repo. Docs in `timbot_gui/docs/README_GUI.md`.

---

## 6. Package build types (know this before editing CMake)
- **One `ament_python` package:** `timbot_gui` (has `setup.py`, console_scripts entry point).
- **Everything else is `ament_cmake`**, even the Python nodes. Python scripts are installed via `install(PROGRAMS src/foo.py DESTINATION lib/${PROJECT_NAME} RENAME foo)` and run with `ros2 run <pkg> foo.py`. Shared Python modules imported by a node (e.g. `classical_lane_detection.py`, `ml_lane_detection.py`) are installed with `install(FILES ...)` to the same `lib/<pkg>` dir so `import` works. When adding a Python node to a CMake package, you **must** add it to `CMakeLists.txt` — there is no automatic discovery.
- CMake Python packages contain a `if(DEFINED ENV{CONDA_PREFIX})` block that points `Python3_ROOT_DIR` at the conda env. This is why a stale conda env breaks the build.

---

## 7. Key topics (contract between modules)
| Topic | Type | Producer → Consumer |
|---|---|---|
| `/cmd_vel` | Twist | twist_mux → motor_control / Gazebo |
| `/nav_vel`,`/joy_vel`,`/key_vel` | Twist | Nav2 / joy / keyboard → twist_mux |
| `/wheel_odom` | Odometry | motor_odom (or Gazebo) → ekf_local |
| `/imu/data` | Imu | phidgets / Gazebo → ekf_local, cartographer, navsat |
| `/scan_lower` (`/scan_upper`) | LaserScan | rplidar / Gazebo → filter_lidar |
| `/scan_modified` | LaserScan | filter_lidar → cartographer, costmaps |
| `/gps/fix` → `/gps/fix_cov` | NavSatFix | navsat driver → gps_cov_relay → navsat_transform/cartographer |
| `/zed_node/left/{image,depth_image,camera_info,points}` | Image/PC2 | zed_open_capture_node or Gazebo bridge → CV |
| `/cv/lane_detections_cloud` | PointCloud2 | lane_detection → cartographer, costmaps |
| `/zed_node/left/obstacle_points` | PointCloud2 | depth_detection → cartographer, costmaps |
| `/zed_node/left/ramp_points` | PointCloud2 | depth_detection → ramp_navigate (visualization) |
| `/ramp_seg` | PoseArray | filter_lidar (if `ramp_seg_using_lidar: true`) OR depth_detection (if `false`) → ramp_navigate |
| `/tracked_pose` → `/tracked_pose_cov` | Pose(Cov)Stamped | cartographer → pose_relay → ekf_global |
| `/odometry/local`, `/odometry/global`, `/odometry/gps` | Odometry | EKFs / navsat |
| `/map` | OccupancyGrid | cartographer occupancy grid → Nav2 |
| `/ramp_naving`, `/waypoint_int`, `pause_navigation` | Bool | mission coordination |

---

## 8. Gotchas & sharp edges (high-value to know)
- **`use_sim_time` is plumbed as the launch arg named `sim`** through every launch file; sim/real branching keys off it everywhere. Don't rename it casually.
- **Hardcoded machine paths:** `.vscode/settings.json` (`/home/harsh/projects/timbot`) is specific to a particular machine. The repo itself lives at `/home/czarhc/projects/timbot` on this machine but is cloned to `~/timbot` on the rover/Pi.
- **`cyclonedds.xml`** pins the DDS network interface to `enx9c69d349d5f9` (a specific USB-Ethernet adapter for the laptop↔Pi link). On other machines set `CYCLONEDDS_URI`/interface appropriately or DDS discovery may fail.
- **Sim ↔ real topic parity is intentional:** Gazebo bridges/plugins and the real drivers publish the *same* topic names (`/scan_lower`, `/imu/data`, `/wheel_odom`, `/zed_node/left/*`) so the entire downstream stack is mode-agnostic. The `zed_open_capture_node` exists specifically to honor this contract on hardware.
- **Two absolute pose sources (GPS + SLAM)** feed `ekf_global`; mis-tuning causes the "rover fidgeting at obstacles / cartographer jitter" issues that recur throughout the git history. Treat odom + cartographer + nav tuning as coupled.
- **Many nav2 configs and a few odom configs are dead/experimental.** The live one is always whatever the active YAML's `config_file:` names — trace that, don't guess from filenames.
- The repo root has scratch artifacts (`rosbag2_*`, `frames_*.pdf/.gv`, `ZED_Diagnostic_Results.json`, `setup.pdf`, `UTRA_ROVERSCHEMATIC.zip`). `*.mp4/*.mov`, `build/`, `install/`, `log/`, `__pycache__/`, `.vscode/` are gitignored.

---

## 9. Workflow conventions
- Branching/PR conventions are in `CONTRIBUTING.md` (feature/bugfix/hotfix branches off `main`; `main` is protected). The active development branch at time of writing is `comp_leadup`. Recent history is competition-prep tuning (nav2, GPS heading via datum, ZED denoising, cartographer weights).
- Hardware port discovery (when comp.yaml ports are wrong) — the commands are documented inline in `comp.yaml`: `v4l2-ctl --list-devices` (ZED), the `udevadm` loop over `/dev/ttyUSB*` (serial), `guvcview -d /dev/video2` (camera check), `refresh_motor` (re-kick the ROS daemon / motor link).
- See also `commands.txt` (quick cheat-sheet) and `README_BUILD.md` (conda/empy build troubleshooting).
