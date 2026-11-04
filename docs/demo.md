# Running the lidar patrol demo

`scripts/demo_patrol_lidar.sh` starts the whole simulation with one command: a
simulated drone with a 2D lidar takes off, patrols a 10 m square while steering
round obstacles, and builds a live map you can watch in RViz.

The demo **saves nothing**: no map, no objects CSV, no bag recording. To keep a
map, run the steps by hand instead ([running_sim.md](running_sim.md)).

## 1. Before the first run

These are needed once per computer. Versions are in [setup.md](setup.md).

**Installed software**

| What | Where the script expects it |
|---|---|
| Ubuntu 24.04 with a desktop (the script opens `gnome-terminal` windows) | |
| ROS 2 Jazzy, plus `ros-jazzy-ros-gz-bridge` | `/opt/ros/jazzy` |
| Gazebo Harmonic | system install |
| PX4-Autopilot v1.17.0, built for SITL | `~/dev/PX4-Autopilot` |
| `px4_msgs` workspace, built | `~/dev/px4_ws` |
| Micro XRCE-DDS Agent (`MicroXRCEAgent` on `PATH`) | |
| QGroundControl AppImage | `~/dev/tools/QGroundControl-x86_64.AppImage` |

If yours are somewhere else, see [Options](#4-options).

**Build this repo**

```bash
cd ~/dev/DroneSwarm-MAP-SAR
source /opt/ros/jazzy/setup.bash
source ~/dev/px4_ws/install/setup.bash
colcon build --packages-select las_offboard
```
Rebuild after pulling new code.

**Link the worlds into PX4** (PX4 only loads worlds from its own folder)

```bash
for w in lidar_test forest_pillars; do
  ln -s ~/dev/DroneSwarm-MAP-SAR/src/las_offboard/worlds/$w.sdf \
        ~/dev/PX4-Autopilot/Tools/simulation/gz/worlds/$w.sdf
done
```

**Let PX4 arm without a radio controller.** A real drone needs stick input before
it will arm; the sim has none. Pick one:
- In QGroundControl: open Application Settings (Q logo, top left) and turn on
  **Virtual Joystick**. It's under **Fly View** in newer versions, **General** in
  older ones.
- Or tell the simulated drone not to need sticks. During a run, type this in the
  **PX4** window at the `pxh>` prompt (PX4 remembers it for later runs):
  ```
  param set COM_RC_IN_MODE 4
  ```
  To undo: `param set COM_RC_IN_MODE 0`.

## 2. Start the demo

```bash
cd ~/dev/DroneSwarm-MAP-SAR
scripts/demo_patrol_lidar.sh
```

The script opens seven windows, waiting for each part to be ready before starting
the next, and prints its progress:

```
Starting patrol_lidar demo in 'forest_pillars' (nothing will be saved)
  waiting for Gazebo world... ok
  waiting for PX4... ok
  waiting for lidar on /scan... ok
  waiting for QGroundControl link... ok
Flying. Watch RViz. Stop with: scripts/demo_patrol_lidar.sh stop
```

The first start takes about a minute. If a step says `timed out`, look at that
part's window for the error and see [Troubleshooting](#6-troubleshooting).

## 3. What you'll see

| Window | What it shows |
|---|---|
| **Gazebo** | The 3D world and the drone. Brown cylinders are tree trunks. |
| **RViz** | Top-down live map: black = obstacle, white = free, grey = not seen yet. Coloured markers are the objects the drone has found, labelled `#id kind`. |
| **QGroundControl** | The drone's status, flight mode and battery, as a ground station sees it. |
| **patrol_lidar** | What the drone is doing: takeoff, laps, obstacles avoided, objects found. |
| DDS agent, PX4, Lidar bridge | Background plumbing. Only needed if something goes wrong. |

Object colours in RViz:

| Colour | Kind | Means |
|---|---|---|
| Blue | post | Small: a trunk or pole |
| Red | wall | Long and thin |
| Dark green | tree | Large irregular clump, e.g. a tree canopy |
| Orange | object | Anything else, e.g. a crate |

The drone climbs to 2 m, flies laps of the square, and flies home along the square
when the battery reaches 35%, then lands. A full flight is several minutes.

## 4. Options

**Choose the world** by naming it:

```bash
scripts/demo_patrol_lidar.sh forest_pillars   # default: 30 trunks in a 15 x 15 m area
scripts/demo_patrol_lidar.sh lidar_test       # a few walls, pillars and crates
scripts/demo_patrol_lidar.sh forest           # PX4's forest with real tree models
```
The `forest` world downloads its tree models the first time, so it needs internet
and starts more slowly once.

**Change paths or altitude** with variables in front of the command:

```bash
QGC=~/Downloads/QGroundControl.AppImage scripts/demo_patrol_lidar.sh
ALT=3.0 scripts/demo_patrol_lidar.sh                  # patrol altitude, metres
PX4_DIR=~/PX4-Autopilot PX4_WS=~/px4_ws scripts/demo_patrol_lidar.sh
```
Keep the altitude below the obstacles' height or the lidar flies over them
(`lidar_test` obstacles are 3 m tall; trunks are 6–10 m).

`scripts/demo_patrol_lidar.sh --help` prints a short summary.

## 5. Stop the demo

```bash
scripts/demo_patrol_lidar.sh stop
```
This stops every part and closes all the demo windows. Use it even if the drone
has already landed, and before starting another run. The script refuses to start
while a previous simulator is still running.

## 6. Troubleshooting

| Problem | Fix |
|---|---|
| `A simulator is already running` | Run `scripts/demo_patrol_lidar.sh stop`, then start again. |
| `World '...' not found` | Link the world into PX4 (section 1). |
| `Workspace not built` | Build this repo (section 1). |
| `waiting for Gazebo world... timed out` | Check the Gazebo window. "Timed out waiting for Gazebo world" there usually means ROS is mixed into Gazebo's setup; the script removes it, so check that nothing else in your `~/.bashrc` adds a different `gz`. |
| `waiting for QGroundControl link... timed out` | QGroundControl didn't open or didn't connect. Check the `QGC=` path, and that QGroundControl shows the drone (top bar). |
| patrol_lidar: `Could not arm or enter offboard` | PX4 wants stick input: turn on QGC's virtual joystick or set `COM_RC_IN_MODE 4` (section 1). Then stop and start the demo. |
| patrol_lidar: `More than one publisher on ...` | Something else is publishing the drone's topics, usually a `ros2 bag play`. Stop it; the drone won't take off until it's gone. |
| RViz prints `GLSL link result ... active samplers` | Harmless graphics-driver warning; the map still draws. |
| RViz shows no map | The map appears once the drone reaches patrol altitude; wait for takeoff to finish. |

More detail on each part: [running_sim.md](running_sim.md).

## Note on PX4 logs

PX4 writes its own flight log (`.ulg`) under
`~/dev/PX4-Autopilot/build/px4_sitl_default/rootfs/log/` on every simulator run,
including the demo. That's PX4's logging, not the demo's results. Delete the files
if you don't want them.
