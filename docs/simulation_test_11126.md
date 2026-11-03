# Simulation test prep: 2026-11-01

Checklist for the LIDAR simulation test session. Base commands are in
[running_sim.md](running_sim.md); this file covers what to prepare and watch for.

Status when written (2026-10-31):

| Item | State |
|---|---|
| PX4 SITL build (`~/dev/PX4-Autopilot`, v1.17.0) | Built |
| `px4_msgs` workspace (`~/dev/px4_ws`) | Built |
| `MicroXRCEAgent` | Installed (`/usr/local/bin`) |
| `lidar_test.py` (reads lidar from Gazebo directly) | Tested, works in `default` and `lidar_test` worlds |
| `lidar_test` world (`src/las_offboard/worlds/lidar_test.sdf`) | Tested, linked into PX4 worlds folder |
| `ros-jazzy-ros-gz-bridge` | **Not installed** |
| `lidar_bridge.launch.py` (lidar → ROS 2 `/scan`) | **Untested** (needs the bridge) |
| QGroundControl | **Not installed** |

---

## 1. Tonight

### 1.1 Install the bridge and test it

```bash
sudo apt install -y ros-jazzy-ros-gz-bridge
source /opt/ros/jazzy/setup.bash
source ~/dev/px4_ws/install/setup.bash
cd ~/dev/DroneSwarm-MAP-SAR
colcon build --packages-select las_offboard
```

Test:

```bash
# Terminal 1
cd ~/dev/PX4-Autopilot
PX4_GZ_WORLD=lidar_test make px4_sitl gz_x500_lidar_2d

# Terminal 2
source /opt/ros/jazzy/setup.bash
source ~/dev/DroneSwarm-MAP-SAR/install/setup.bash
ros2 launch las_offboard lidar_bridge.launch.py world:=lidar_test

# Terminal 3
ros2 topic hz /scan          # expect ~30 Hz
ros2 topic echo --once /scan
rviz2                        # Fixed Frame: link, Add > By topic > /scan > LaserScan
```

- [ ] Bridge installed
- [ ] `/scan` shows ~30 Hz
- [ ] Obstacles visible in RViz

### 1.2 Install QGroundControl

Download the Linux AppImage from <https://qgroundcontrol.com>, then:

```bash
chmod +x QGroundControl*.AppImage
./QGroundControl*.AppImage
```

It shows the drone's mode, warnings and battery, and lets you take over manually
if a node misbehaves.

- [ ] QGC installed and connects to the sim

### 1.3 Do one full end-to-end run

Run everything together once and note which terminal runs what:

| Terminal | Command |
|---|---|
| 1 | `MicroXRCEAgent udp4 -p 8888` |
| 2 | `cd ~/dev/PX4-Autopilot && PX4_GZ_WORLD=lidar_test make px4_sitl gz_x500_lidar_2d` |
| 3 | `ros2 launch las_offboard lidar_bridge.launch.py world:=lidar_test` |
| 4 | `python3 ~/dev/DroneSwarm-MAP-SAR/src/las_offboard/las_offboard/lidar_test.py` |
| 5 | `ros2 run las_offboard patrol_rtl --ros-args -p altitude_m:=2.0 -p patrol_size_m:=14.0` |

Terminals 3 and 5 need the sourcing from [running_sim.md](running_sim.md) section 2.
Optional: a small `tmux` script that opens all of these would save time tomorrow.

- [ ] Full run works end to end

---

## 2. Before flying in the obstacle world

### 2.1 Fly low enough to see obstacles

The lidar scans flat from the top of the drone. The test obstacles are 3 m tall,
and `takeoff_land` / `patrol_rtl` fly at 5 m by default, so at 5 m the lidar scans
over everything and reports "clear". Fly at 2 m:

```bash
-p altitude_m:=2.0
```

### 2.2 Widen the patrol, or the drone will hit the red wall

`patrol_rtl` flies a square centred on the spawn point (default 10 m wide, so
corners at ±5 m). Obstacle positions (Gazebo x = East, y = North, metres):

| Obstacle | Centre | Size |
|---|---|---|
| Red wall | (5, 0) | 0.3 × 6 m, spans y = -3 to 3 |
| Blue pillar | (0, 3) | radius 0.3 m |
| Green crate | (1.5, -2) | 1 × 1 m |
| Yellow wall | (10, 10) | 6 × 0.3 m, spans x = 7 to 13 |
| Purple post | (-1, -1.2) | 0.5 × 0.5 m |

| `patrol_size_m` | Square edges at | Result |
|---|---|---|
| 10 (default) | ±5 m | **East edge hits the red wall** |
| 6 | ±3 m | **North edge hits the blue pillar** |
| 14 | ±7 m | Clear, sees obstacles from outside |

```bash
ros2 run las_offboard patrol_rtl --ros-args -p altitude_m:=2.0 -p patrol_size_m:=14.0
```

**Watch the first run in Gazebo.** The nodes have no obstacle avoidance yet.

### 2.3 The lidar's directions turn with the drone

The lidar's "front" is the drone's nose. The world file's directions assume the
nose points east (+x), as it does at spawn.

- `patrol_rtl` holds the spawn heading, so the directions stay as in the world file.
- `takeoff_land` sets the heading to north (yaw 0), so "front" is about 90° to the left
  of the world-file directions (front = north, left = west, right = east).

---

## 3. Decide what you're testing

Pass/fail checks for the session:

- [ ] `/scan` arrives at ~30 Hz while flying
- [ ] The red wall reads close to its real distance from where the drone is
- [ ] `lidar_test.py` and `/scan` agree on the closest obstacle
- [ ] `patrol_rtl` still returns home at the battery threshold with the lidar drone
      (set `param set SIM_BAT_MIN_PCT 10` at `pxh>` so the battery can drain past 50%)

Record the data:

```bash
mkdir -p ~/dev/DroneSwarm-MAP-SAR/recordings && cd ~/dev/DroneSwarm-MAP-SAR/recordings
ros2 bag record /scan /fmu/out/vehicle_local_position_v1
```

`recordings/` and bag files are already in `.gitignore`.

---

## 4. Also useful

### PX4's built-in collision prevention

PX4 uses the same lidar data for its own collision prevention. At the `pxh>` prompt:

```
param set CP_DIST 2.0
```

It applies in **Position mode** (fly with QGC's virtual joystick or a gamepad), not
offboard. It gives a baseline before writing your own avoidance.

### Common problems

| Symptom | Fix |
|---|---|
| Bridge running but `/scan` has no data | `world:=` doesn't match the world PX4 started. Check with `gz topic -l \| grep scan` |
| `Package 'las_offboard' not found` / `ModuleNotFoundError` | Missing `source` step, or forgot `colcon build` |
| Sim won't start / old drone still there | Leftover processes: `pkill -f px4; pkill -f "gz sim"` |
| Lidar always "clear" in flight | Flying above the 3 m obstacles. Use `altitude_m:=2.0` |
| `PX4_GZ_WORLD=lidar_test` not found | Symlink missing. See [running_sim.md](running_sim.md) section 4 |

More in the troubleshooting table in [running_sim.md](running_sim.md).

### Commit

- [ ] Commit tonight's changes (corrected `lidar_test.sdf` comment, this file, any
      fixes from the bridge test)
