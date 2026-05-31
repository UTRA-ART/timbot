# Simple Nav Testing Plan

## Goal

Validate the rover's **local odometry pipeline** (IMU + wheel encoders + EKF local) and the **open-loop waypoint follower** (`simple_nav_node`) in isolation — without Nav2, Cartographer, GPS, LiDAR, or cameras running. This gives a clean signal on whether the fundamental drive-and-turn behavior works before adding layers.

---

## What this launch runs

```
simple_nav.launch.py
  ├── robot_state_publisher   (URDF → static TF tree: base_link, imu_link, …)
  ├── phidgets_spatial        (raw IMU on /imu/data_raw)
  ├── imu_relay               (NED→ENU correction → /imu/data)
  ├── motor_control_node      (GPIO PWM + Arduino serial encoder read)
  ├── wheel_odom_pub          (encoder ticks → /wheel_odom)
  ├── ekf_local               (fuses /wheel_odom + /imu/data → /odometry/local)
  └── simple_nav_node         (reads /odometry/local, publishes /cmd_vel)
```

Nothing else. The rover is fully open-loop with respect to the environment: it will not avoid obstacles, use a map, or correct via GPS.

---

## Pre-flight checklist

- [ ] Arduino is connected and running `read_hall.ino` (`/dev/ttyACM0`)
- [ ] Phidgets IMU is connected over USB
- [ ] `colcon build --symlink-install --packages-select simple_nav odom_state motor_control motor_odom rover_description` has been run
- [ ] `source install/setup.bash`
- [ ] Open space of at least 3 m × 3 m is clear of obstacles
- [ ] A kill method is ready (`Ctrl-C` on the launch, or hold the manual joystick override — `joy_vel` has priority 100 in twist_mux, so plugging in the PS4 controller instantly stops autonomous motion)

---

## Running

```bash
ros2 launch simple_nav simple_nav.launch.py
```

Optional arguments:

| Argument | Default | Purpose |
|---|---|---|
| `yaw_offset` | `0.0` | Heading calibration offset (degrees) if IMU has a known bias |
| `orientation_stddev` | `0.05` | IMU orientation noise (rad), tighten if heading is noisy |
| `log_level` | `info` | Set to `debug` to see EKF innovation and control steps |

---

## Test sequence

### 1. Odometry sanity (no motion)

**Goal:** confirm `/odometry/local` is being published and is reasonable at rest.

```bash
ros2 topic hz /odometry/local          # should be ~30 Hz
ros2 topic echo /odometry/local --once # pose should be near (0,0,0)
ros2 topic echo /imu/data --once       # orientation quaternion should be ~identity (ENU)
```

Expected:
- `/odometry/local` arrives at ~30 Hz with pose near origin and low covariance.
- `/imu/data` orientation is consistent with the physical orientation of the rover.

---

### 2. Straight-line drive: 2 m forward

Edit `config/simple_nav.yaml`:

```yaml
goals: "[[2.0, 0.0, 0.0]]"
```

Relaunch. Observe:

- [ ] Rover turns to face the goal (or skips turn if already facing it)
- [ ] Drives forward and stops within `position_tolerance` (default 0.10 m)
- [ ] `/simple_nav_node/done` publishes `true` after completion
- [ ] Physically measure travelled distance with a tape — compare to 2.0 m

**Pass criterion:** physical distance within ±15 cm of 2.0 m.

---

### 3. 90-degree turn in place

Edit goals:

```yaml
goals: "[[0.0, 0.0, 1.5707963]]"   # 90 deg left
```

- [ ] Rover rotates ~90° and stops
- [ ] Heading on `/odometry/local` after the turn matches expected yaw (`~1.57 rad`)

**Pass criterion:** final yaw within ±5° of 90°.

---

### 4. L-shaped path (2 m forward + 90° turn + 1 m forward)

This is the default config:

```yaml
goals: "[[2.0, 0.0, 0.0], [0.0, 0.0, 1.5707963], [1.0, 0.0, 0.0]]"
```

Mark the start position with tape. After completion:

- [ ] Measure the L shape on the ground
- [ ] Start→corner segment ≈ 2 m
- [ ] Corner→end segment ≈ 1 m, perpendicular to the first

**Pass criterion:** segments within ±20 cm; heading at each waypoint within ±8°.

---

### 5. Closed square (4 × 90° turns)

```yaml
goals: "[[2.0, 0.0, 1.5707963], [2.0, 0.0, 1.5707963], [2.0, 0.0, 1.5707963], [2.0, 0.0, 1.5707963]]"
```

- [ ] Rover completes a roughly square loop
- [ ] End position is within ~0.5 m of start (odometry drift over ~10 m is expected)

This test exposes cumulative drift from the encoders and IMU. Record the final odometry position to establish a drift baseline.

---

## Failure modes and what they indicate

| Symptom | Likely cause |
|---|---|
| `/odometry/local` not published | EKF not receiving `/wheel_odom` or `/imu/data` — check motor_control / phidgets |
| Rover drives correct distance but wrong heading | IMU yaw bias — tune `yaw_offset` arg |
| Rover immediately overshoots turn | `kp_angular` too high or `angular_tolerance` too loose |
| Rover curves during straight drive | Left/right encoder scaling mismatch — check `convert_speed_left/right` in `motor_control.py` |
| Rover stops short or overshoots distance | `WHEEL_RADIUS` or `WHEEL_BASE` in `odom_pub.py` needs re-measurement |
| EKF covariance blows up | Angular velocity from IMU inconsistent with wheel yaw rate — likely `imu0_angular_velocity_noise` needs tuning in `odom.yaml` |

---

## Key topics to monitor during testing

```bash
# Control output
ros2 topic echo /cmd_vel

# Fused odometry (the feedback signal)
ros2 topic echo /odometry/local

# Raw encoder inputs
ros2 topic echo /left_wheel/ticks
ros2 topic echo /right_wheel/ticks

# Completion signal
ros2 topic echo /simple_nav_node/done
```

---

## What comes next

Once this baseline passes:

1. **Add twist_mux** — verify manual joystick override works during autonomous motion
2. **Add Cartographer** — verify `map→odom` TF appears and `/odometry/local` feeds into it correctly
3. **Add GPS + ekf_global** — test global anchoring without Nav2
4. **Full stack** — re-run the L-shape test with the complete `comp.yaml` launch and compare
