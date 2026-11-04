# Landing accuracy check

Measures how close to the landing pad the drone came to rest, against the pad's
true position in the Gazebo world file. Use it after changing precision landing
(`pad_detector`, or `PRECISION_LAND` in `patrol_lidar`).

## 1. Record the landing

Run the precision-landing setup ([docs/running_sim.md](../../docs/running_sim.md),
"Precision landing on the pad") and in another terminal (ROS sourced):

```bash
cd ~/dev/DroneSwarm-MAP-SAR/tools/land_eval
ros2 bag record -o ~/land_1 /fmu/out/vehicle_local_position_v1 /landing_pad \
    --qos-profile-overrides-path ../avoid_eval/qos.yaml
```
Stop it with Ctrl+C after the drone has disarmed.

## 2. Measure

Pass the world the flight was in, then one or more recordings:

```bash
python3 landing_error.py ~/dev/DroneSwarm-MAP-SAR/src/las_offboard/worlds/forest_pillars_moved_pad.sdf \
    ~/land_1 ~/land_2
```

```
pad centre: E=+1.00 N=+0.60 m from the spawn point
flight                     touchdown E, N    error  on pad  took off          pad estimates
land_mbase                 -0.11,  -0.00  126.3 cm      NO    1.15 m      17, off by 4.2 cm
land_m1                    +0.98,  +0.63    3.5 cm     yes    1.17 m     118, off by 1.9 cm
```

| Column | Meaning |
|---|---|
| error | Drone centre at rest to pad centre |
| on pad | Drone centre within the 0.5 m pad |
| took off | How far the pad was from where the drone took off |
| pad estimates | How many `/landing_pad` messages, and how far their average was from the true centre |

With several recordings it also prints the mean and worst error. The pad is
found by name (`landing_pad`) in the world file; the drone is assumed to have
spawned at the world origin (`--spawn E N` if not).

## Results (2026-11-04)

| World | Landing | Touchdown error |
|---|---|---|
| `forest_pillars` (pad at take-off) | normal | 11.1 cm |
| `forest_pillars` (pad at take-off) | precision, 3 flights | 9.7–10.0 cm |
| `forest_pillars_moved_pad` (pad 1.17 m away) | normal | 126.3 cm (off the pad) |
| `forest_pillars_moved_pad` | precision, 2 flights | 3.5, 7.1 cm |
| `forest_pillars_random_pad`, dock position 0.8 m wrong | precision, 2 random spots 6.7 and 4.9 m away | 6.2, 4.3 cm |
