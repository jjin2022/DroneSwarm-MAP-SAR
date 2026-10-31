# Running the simulation

Commands to bring up PX4 SITL + Gazebo and fly the `las_offboard` nodes.
Versions are listed in [setup.md](setup.md).

Paths assumed:

| What | Path |
|---|---|
| PX4 | `~/dev/PX4-Autopilot` |
| px4_msgs workspace | `~/dev/px4_ws` |
| This repo | `~/dev/DroneSwarm-MAP-SAR` |

## 1. Build the workspace (after pulling or changing code)

```bash
source /opt/ros/jazzy/setup.bash
source ~/dev/px4_ws/install/setup.bash
cd ~/dev/DroneSwarm-MAP-SAR
colcon build --packages-select las_offboard
```

## 2. Single drone

Use one terminal per step.

**Terminal 1: DDS agent** (one agent serves every drone)
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2: PX4 + Gazebo**
```bash
cd ~/dev/PX4-Autopilot
make px4_sitl gz_x500
```
Wait for the `pxh>` prompt. Optional, to let the simulated battery drain past 50%:
```
param set SIM_BAT_MIN_PCT 10
```

**Terminal 3: node**
```bash
source /opt/ros/jazzy/setup.bash
source ~/dev/px4_ws/install/setup.bash
source ~/dev/DroneSwarm-MAP-SAR/install/setup.bash
ros2 run las_offboard patrol_rtl          # or: takeoff_land
```

The single drone is PX4 instance 0: topics are `/fmu/...`, no namespace.

## 3. Multiple drones

Stop any `make px4_sitl` first. Each drone is a separate PX4 instance; instance `N`
publishes on `/px4_N/fmu/...` and has MAVLink system ID `N+1`.

**Terminal 1: DDS agent**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2: drone 1** (also starts Gazebo)
```bash
cd ~/dev/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 \
  ./build/px4_sitl_default/bin/px4 -i 1
```

**Terminal 3: drone 2** (start once Gazebo is up)
```bash
cd ~/dev/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 \
  PX4_GZ_MODEL_POSE="0,15" ./build/px4_sitl_default/bin/px4 -i 2
```
More drones: repeat with `-i 3`, `-i 4`, ... and a different `PX4_GZ_MODEL_POSE` (x,y in metres).
Keep spawns at least 15 m apart: each drone patrols a 10 m square around its own spawn point.

Each instance has its own parameters, so set `SIM_BAT_MIN_PCT` at every `pxh>` prompt.

**Terminals 4+: one node per drone**
```bash
source /opt/ros/jazzy/setup.bash
source ~/dev/px4_ws/install/setup.bash
source ~/dev/DroneSwarm-MAP-SAR/install/setup.bash
ros2 run las_offboard patrol_rtl --ros-args -r __ns:=/px4_1
```
```bash
# same sourcing as above
ros2 run las_offboard patrol_rtl --ros-args -r __ns:=/px4_2 -p altitude_m:=8.0
```
The node reads the MAVLink system ID from `VehicleStatus`; pass `-p target_system:=N` to override.

## 4. LIDAR (x500 with 2D lidar)

One-time setup:
```bash
sudo apt install ros-jazzy-ros-gz-bridge
# PX4 only loads worlds from its own folder, so link ours in:
ln -s ~/dev/DroneSwarm-MAP-SAR/src/las_offboard/worlds/lidar_test.sdf \
      ~/dev/PX4-Autopilot/Tools/simulation/gz/worlds/lidar_test.sdf
```

**Terminal 1: PX4 + Gazebo** (`PX4_GZ_WORLD` picks the world; omit it for the empty `default` world)
```bash
cd ~/dev/PX4-Autopilot
PX4_GZ_WORLD=lidar_test make px4_sitl gz_x500_lidar_2d
```

**Terminal 2: either** read the lidar straight from Gazebo (no ROS needed)
```bash
python3 ~/dev/DroneSwarm-MAP-SAR/src/las_offboard/las_offboard/lidar_test.py
```
**or** bridge it into ROS 2 as `/scan` (`sensor_msgs/LaserScan`, frame `link`)
```bash
# same sourcing as section 2
ros2 launch las_offboard lidar_bridge.launch.py world:=lidar_test
ros2 topic hz /scan          # ~30 Hz
rviz2                        # Fixed Frame: link, Add > By topic > /scan
```
The `world:=` value must match the world PX4 started (`default` if you didn't set `PX4_GZ_WORLD`).

## 5. Checks

```bash
ros2 topic list | grep fmu/out                       # topics PX4 is exposing
ros2 topic info /fmu/out/vehicle_status_v1           # Publisher count should be 1
ros2 topic echo --once --qos-reliability best_effort /fmu/out/battery_status_v1
```
For multi-drone, prefix topics with `/px4_N`.

## 6. `patrol_rtl` parameters

| Parameter | Default | Meaning |
|---|---|---|
| `altitude_m` | 5.0 | Patrol altitude above home |
| `patrol_size_m` | 10.0 | Side of the patrol square |
| `return_battery` | 0.35 | Return home at or below this fraction |
| `reach_tolerance_m` | 0.5 | Distance that counts as reaching a waypoint |
| `target_system` | 0 | MAVLink system ID; 0 = auto from `VehicleStatus` |

Set with `--ros-args -p name:=value`.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `Waiting for PX4 status, local position and battery...` forever | Node namespace doesn't match a running PX4 instance. `make px4_sitl` is instance 0 (no namespace); `-i N` is `/px4_N`. Also check the agent is running. |
| Battery stuck at 50%, drone never returns | `SIM_BAT_MIN_PCT` defaults to 50. Lower it, or run with `-p return_battery:=0.6`. |
| `No offboard signal` in QGC after landing | Expected: the node stops streaming once PX4 is landing. |
| Topic listed but no data | PX4 1.17 publishes versioned topics (`_v1`). Check `Publisher count` with `ros2 topic info`. |
| `ModuleNotFoundError: px4_msgs` | Source `~/dev/px4_ws/install/setup.bash`. |
