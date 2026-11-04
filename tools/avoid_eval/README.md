# Obstacle clearance check

Measures how close the drone really got to obstacles during a simulated flight,
using the drone's recorded position and the obstacle shapes in the Gazebo world
file. This is ground truth: it doesn't depend on what the lidar or the map saw.
Use it after changing `patrol_lidar`'s avoidance (`free_heading`, `plan_to`).

## 1. Record a flight

Run the sim and `patrol_lidar` as usual ([docs/running_sim.md](../../docs/running_sim.md)),
and in another terminal (ROS sourced) record the drone's position:

```bash
cd ~/dev/DroneSwarm-MAP-SAR/tools/avoid_eval
ros2 bag record -o ~/flight_forest /fmu/out/vehicle_local_position_v1 \
    --qos-profile-overrides-path qos.yaml
```
Stop it with Ctrl+C after a few laps. `qos.yaml` is needed: PX4 publishes
best-effort, and without it the recorder gets nothing.

## 2. Measure

```bash
python3 clearance.py ~/flight_forest \
    ~/dev/DroneSwarm-MAP-SAR/src/las_offboard/worlds/forest_pillars.sdf
```

```
8598 samples over 194 s in the air
closest approach: 0.59 m to trunk_23  (propeller tips at ~0.38 m: below that is contact)
  time closer than 0.38 m:   0.0 s
  time closer than 0.50 m:   0.0 s
  time closer than 0.80 m:   4.6 s

9 passes closer than 0.8 m (bearing: obstacle direction vs travel, 0 = ahead, 90 = beside, 180 = behind)
  0.59 m  t=  17.5s  trunk_23       drone at E= -6.2 N=  2.2  0.2 m/s  bearing   75 deg  for 0.8 s
  ...
```

- **Closest approach** is from the drone's centre to the obstacle's surface. The
  x500's propeller tips reach about 0.38 m, so below that is contact.
- **0.8 m** is `patrol_lidar`'s `avoid_radius_m`; change it with `--clearance`.
- **Bearing** says where the obstacle was: about 0° means the drone flew at it,
  about 90° that it brushed past it.
- If the recorder dropped messages, a `note:` line says how much time is missing;
  the times cover only the recorded part.

Works for any world made of cylinders and boxes (`forest_pillars`, `lidar_test`).
Assumes the drone spawned at the world origin, as PX4 does by default.

## Results so far (forest_pillars, ~3 min at 2 m, 2026-11-03)

| Version | Closest | Time < 0.8 m | Passes < 0.8 m |
|---|---|---|---|
| Before the avoidance fixes | 0.47 m | 17.2 s | 26 |
| Path check covers obstacles beside the drone | 0.52 m | 4.9 s | 7 |
| + commit to one side when going round | 0.56–0.59 m | 3.9–4.6 s | 7–9 |

`lidar_test` after the fixes: closest 0.86 m, never inside 0.8 m.
