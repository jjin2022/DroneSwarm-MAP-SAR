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

### `patrol_lidar`: patrol with avoidance, mapping and object tracing

Same patrol as `patrol_rtl`, plus:
- steers round obstacles the lidar sees
- builds a map (`/map`)
- traces objects (`/objects`)
- on low battery, returns **along the square** instead of cutting across

Needs the lidar bridge from above running.

```bash
# same sourcing as section 2
ros2 run las_offboard patrol_lidar --ros-args -p altitude_m:=2.0 \
  -p map_dir:=~/dev/DroneSwarm-MAP-SAR/recordings
rviz2 -d $(ros2 pkg prefix las_offboard)/share/las_offboard/rviz/patrol_lidar.rviz   # map + objects
```
At the end it logs the objects found and saves `lidar_map_<time>.pgm/.yaml` and
`lidar_map_<time>_objects.csv` to `map_dir`.

| Parameter | Default | Meaning |
|---|---|---|
| `cruise_speed_m_s` | 2.0 | Approximate flying speed |
| `avoid_radius_m` | 0.8 | Clearance kept from obstacles on each side |
| `avoid_lookahead_m` | 3.0 | How far ahead the path must be clear |
| `stuck_timeout_s` | 15.0 | Skip a corner after this long with no progress |
| `face_travel_direction` | true | Turn the nose (and lidar) toward where it's flying |
| `map_dir` | `.` | Where to save the map. To not save: `-p "map_dir:=''"` (keep the quotes as shown) |

The `patrol_rtl` parameters (`altitude_m`, `patrol_size_m`, `return_battery`, ...) also apply.
Fly at 2 m so the lidar is below the 3 m obstacles. Returning along the square takes
longer than cutting across, so leave battery margin. In the sim,
`param set SIM_BAT_DRAIN 300` at `pxh>` slows the drain.

**Don't replay a bag while the sim is running.** `ros2 bag play` publishes the
recorded flight on the same topics as the live one (`/fmu/out/...`, `/scan`), so a
node gets both mixed together: phantom obstacles and a wrong position.
`patrol_lidar` checks for this once a second. Every topic it reads must have exactly
one publisher (PX4's agent or the lidar bridge). If one has more:
- **Before takeoff:** it refuses to take off and keeps waiting (disarming if it
  already armed) until the extra publisher is gone.
- **In flight:** it hands control to PX4 to land in place.

The error names the topics:
```
More than one publisher on fmu/out/vehicle_status_v1, ..., /scan (is a `ros2 bag play` running?).
Not taking off until only PX4 and the lidar bridge publish them.
```
Find the culprit with `ros2 topic info -v /scan` or `ros2 node list | grep rosbag2_player`,
and stop it (Ctrl+C in its terminal). Replay bags with the sim closed, or on another
`ROS_DOMAIN_ID`. `patrol_rtl` and `takeoff_land` don't have this check.

### `forest_pillars` world: semi-dense forest of plain trunks

30 cylinder trunks (radius 0.12-0.35 m, 6-10 m tall) in a 15 x 15 m square
centred on the spawn point, at least 2 m apart. The spawn point and the corners
of the default 10 m patrol square are kept clear; a few trunks stand on the patrol
path so the drone has to steer round them. Plain cylinders give the lidar clean
returns, unlike PX4's `forest` world, whose leafy tree models break into many
false objects.

One-time link (same as `lidar_test`):
```bash
ln -s ~/dev/DroneSwarm-MAP-SAR/src/las_offboard/worlds/forest_pillars.sdf \
      ~/dev/PX4-Autopilot/Tools/simulation/gz/worlds/forest_pillars.sdf
```
Then run everything in this section with `forest_pillars` instead of `lidar_test`
(`PX4_GZ_WORLD=forest_pillars`, `world:=forest_pillars`).

The world is generated by `src/las_offboard/worlds/gen_forest.py`; don't edit the
`.sdf` by hand. For a different layout or density:
```bash
cd ~/dev/DroneSwarm-MAP-SAR/src/las_offboard/worlds
python3 gen_forest.py --seed 7 --trees 40 --min-gap 1.8   # see --help for all options
```
The same seed and options always give the same forest. Keep `--min-gap` above
about 1.8 m: `patrol_lidar` keeps 0.8 m clear on each side, so narrower gaps
are impassable. If you change `patrol_size_m`, pass the same value as `--patrol`
so its corners stay clear.

### Precision landing on the pad

`forest_pillars` has a landing pad at the spawn point: PX4's 0.5 m ArUco marker
(4×4, ID 0). With a downward camera, `patrol_lidar` lands on it instead of just
flying back to where it took off: it centres over the pad at patrol altitude,
descends only while centred, and at 0.6 m (where the camera can no longer see
the whole marker) hands over to PX4 to land straight down.

One-time setup: link the camera drone model into PX4, and (for the moved-pad
test world) that world too:
```bash
ln -s ~/dev/DroneSwarm-MAP-SAR/src/las_offboard/models/x500_lidar_cam_down \
      ~/dev/PX4-Autopilot/Tools/simulation/gz/models/x500_lidar_cam_down
ln -s ~/dev/DroneSwarm-MAP-SAR/src/las_offboard/worlds/forest_pillars_moved_pad.sdf \
      ~/dev/PX4-Autopilot/Tools/simulation/gz/worlds/forest_pillars_moved_pad.sdf
```

Run it like the lidar patrol above, with three differences:

| Terminal | Change |
|---|---|
| PX4 | Start the binary directly with the camera model (PX4 has no `make` target for it):<br>`cd ~/dev/PX4-Autopilot && PX4_SYS_AUTOSTART=4013 PX4_SIM_MODEL=gz_x500_lidar_cam_down PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=forest_pillars ./build/px4_sitl_default/bin/px4` |
| Lidar bridge | Also bridge the camera:<br>`ros2 launch las_offboard lidar_bridge.launch.py world:=forest_pillars model:=x500_lidar_cam_down_0 camera:=true` |
| New: pad detector | `ros2 run las_offboard pad_detector` (publishes `/landing_pad`; add `/camera/pad_debug` in RViz to see what it detects) |

`patrol_lidar` uses precision landing whenever `pad_detector` is running; without
it, it lands normally. For a short test flight, add `-p return_battery:=0.93`
(heads home after about one lap).

| Parameter | Default | Meaning |
|---|---|---|
| `precision_landing` | true | Use the pad when `pad_detector` is running |
| `land_descent_rate_m_s` | 0.3 | Descent speed while centred |
| `land_align_tolerance_m` | 0.15 | How close to the pad centre counts as centred |
| `land_handoff_height_m` | 0.6 | Height at which PX4 takes over the last part |
| `pad_lost_timeout_s` | 5.0 | Pad not seen this long: normal landing where it is |
| `precision_land_timeout_s` | 60.0 | Whole precision landing longer than this: normal landing |

**Results (2026-11-04)**, touchdown distance from the pad's true centre:

| World | Normal landing | Precision landing |
|---|---|---|
| `forest_pillars` (pad where the drone took off) | 11 cm | 10 cm (3 flights) |
| `forest_pillars_moved_pad` (pad 1.17 m from take-off: a dock that moved) | **126 cm, off the pad** | **3.5 and 7.1 cm** |

With the pad where it took off, the simulator's near-perfect position estimate
already brings the drone back onto it; the ~10 cm both ways is PX4's final
descent. Precision landing matters when the drone doesn't know exactly where
the pad is: a dock that moved, or real GPS drift.

`pad_detector` uses OpenCV. If a NumPy 2 is installed in `~/.local`, Ubuntu's
OpenCV can't load with it; the node detects this and restarts itself with
user packages turned off (`PYTHONNOUSERSITE=1`), so nothing needs changing.

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
| `More than one publisher on ...` from `patrol_lidar` | Something else publishes on the drone's topics, usually a `ros2 bag play`. Stop it; see the warning in section 4. |
| Phantom obstacles, drone dodging nothing or wobbling (`patrol_rtl` / `takeoff_land`) | Same cause: a bag replay mixing in old data. These nodes don't check for it. Stop the replay and land. |

## Testing changes to the object tracer

After changing the tracer or object labels in `patrol_lidar.py`, run
`tools/tracer_eval/evaluate.py`: it replays recorded flights through the tracer and
checks the results against each world's real objects, without the simulator. See
[tools/tracer_eval/README.md](../tools/tracer_eval/README.md).

## Testing changes to obstacle avoidance

After changing avoidance in `patrol_lidar.py`, record a flight's position and run
`tools/avoid_eval/clearance.py`: it measures how close the drone really got to each
obstacle, from the world file. See [tools/avoid_eval/README.md](../tools/avoid_eval/README.md).
