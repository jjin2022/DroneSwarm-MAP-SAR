# Object tracer evaluation

Checks `patrol_lidar`'s object tracer against recorded flights in seconds, without
starting the simulator. Run it after changing `ObjectTracer`, `TracedObject` or
`kind()` in `src/las_offboard/las_offboard/patrol_lidar.py`.

```bash
source /opt/ros/jazzy/setup.bash
source ~/dev/px4_ws/install/setup.bash
cd ~/dev/DroneSwarm-MAP-SAR/tools/tracer_eval
python3 evaluate.py                # current code; exits 1 if a check fails
python3 evaluate.py --rev HEAD~1   # an older commit, to compare
```

Output (2026-11-03, after the tracer and label fixes):

```
dataset             objects observable      found     missed        dup      false    reports     labels
forest                   31         21         18          3         11          2         47      28/29
forest_pillars           30         30         30          0          0          0         30      26/30
lidar_test                5          5          5          0          0          0          5        5/5
```

| Column | Meaning |
|---|---|
| objects | Confirmed objects at the end of the flight |
| observable | Real objects the final map shows (some never come into lidar view) |
| found / missed | Observable objects with / without a traced object |
| dup | Extra traced objects on a real object that's already found |
| false | Traced objects near no real object |
| reports | "Object found" messages over the flight; ideally equal to objects |
| labels | Found objects whose label (post / wall / tree / object) is right |

**Checks:** `forest_pillars` and `lidar_test` must have no missed, duplicate or
false objects. `forest` (PX4's leafy trees) is reported but not checked: the 2D
lidar splits canopies, which the tracer still counts as several objects.

## Datasets

`data/*.npz` hold, for each flight, which map cells were occupied at each of
patrol_lidar's 1 Hz map updates, plus the world's ground truth (object positions
and the label each should get). They're about 1/1000 the size of the bags.

| Dataset | World | Flight |
|---|---|---|
| `lidar_test` | `lidar_test` | ~2 min, 4 laps at 2 m |
| `forest_pillars` | `forest_pillars` (seed 1) | ~3 min, 5 laps at 2 m |
| `forest` | PX4 `forest` | ~2.5 min, 2 m; matched within 3 m of each trunk (canopy) |

All recorded with the mapping code as of commit `8b0d17f` (map starts at patrol
altitude). If the mapping itself changes (`OccupancyMap`, or which scans reach it),
these no longer show what the node would see: record new flights.

## Adding or re-recording a flight

```bash
# while patrol_lidar is flying (see docs/running_sim.md):
ros2 bag record -o ~/my_flight /map
# afterwards:
python3 make_dataset.py ~/my_flight \
    ~/dev/DroneSwarm-MAP-SAR/src/las_offboard/worlds/forest_pillars.sdf \
    data/forest_pillars.npz --radius 1.0
```
`--radius` is how close a traced object must come to a real one to count: 1.0 m
for trunks and boxes, 3.0 m for leafy trees. Ground-truth labels come from each
model's collision shape, using the same size rules as `kind()`.
