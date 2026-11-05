# Sortie runner: the Phase 1 exit test

Flies complete sorties back to back, unattended, and counts how many succeed in a
row. Phase 1 is done at **50 in a row**.

```bash
source /opt/ros/jazzy/setup.bash
source ~/dev/px4_ws/install/setup.bash
source ~/dev/DroneSwarm-MAP-SAR/install/setup.bash
cd ~/dev/DroneSwarm-MAP-SAR
python3 tools/sortie_runner/run_sorties.py                # until 50 in a row (~2 h if all pass)
python3 tools/sortie_runner/run_sorties.py --sorties 2    # a short dry run first
python3 tools/sortie_runner/run_sorties.py --sorties 3 --fault uwb_lost   # check one fallback
```
Results go to `recordings/sorties_<time>/` (git-ignored): `report.csv` (one row per
sortie), `summary.md`, and a folder per sortie with every program's log, a
`runner.log` of timestamped steps, and the recording used for scoring.

## What a sortie is

- No GPS: the drone flies on simulated visual odometry (`sim_vio`, 1% drift).
- It starts on the robot (`forest_pillars_robot`). The robot relocates 10-30 s after
  take-off to a random clear spot in the forest; its own odometry is 3% off.
- About half the sorties get one fault (1/6 each):
  - **radio_drop**: the robot's radio goes silent at a random time (5-45 s);
  - **bad_odometry**: the robot's radio report is 2-4 m wrong;
  - **uwb_lost**: UWB ranging stops at a random time (5-45 s). Added after the first
    run: a good UWB fix comes first in the chain, so without this fault the radio,
    last-known and camera-search fallbacks were never needed.
- The drone patrols until its battery reaches 80%, then returns, finds the robot
  (UWB, radio, last known position, camera search) and lands on it.

## When a sortie passes

All of these (decided for Phase 1):
- **Mission done without a human:** patrolled, returned and landed through precision
  landing: no give-up landing (`GO_HOME`), no handoff, no timeout (10 min).
- **On the pad:** the drone came to rest within the 0.5 m marker, measured against
  the robot's true position, and on the platform (not the ground).
- **No contact:** never closer than 0.38 m (propeller tips) to a trunk, from the true
  positions.

Any failure resets the count.

## How it runs unattended

- Everything starts without windows: Gazebo headless, PX4 without its console
  (`PX4_DAEMON=1` in `scripts/px4_nogps.sh`), no QGroundControl. PX4's x500 setup
  makes "no ground station" block arming, so the runner sets `NAV_DLL_ACT=0` over
  MAVLink after boot. (Passing it at start-up doesn't work: a startup value equal to
  the default is overwritten by the airframe's own default of 2.) Like all GPS-free
  runs, this lands in the PX4 script's own parameter folder.
- Each program runs in its own process group and is stopped (Ctrl-C, then killed)
  after the sortie; the runner checks nothing is left before the next one.
- Topic checks use `ros2 topic list --no-daemon`: the ROS 2 daemon's cache can go
  stale after programs are killed hard and then never show the next sortie's topics.
- It refuses to start while another simulator is running, and stops if your normal
  PX4 parameters ever change.
