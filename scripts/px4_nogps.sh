#!/usr/bin/env bash
# Start PX4 SITL WITHOUT GPS, flying on simulated visual odometry (sim_vio).
#
#   scripts/px4_nogps.sh [world] [spawn pose]   (default world: forest_pillars)
#
#   spawn pose: "x,y,z" (m) where the drone starts; e.g. "0,0,0.46" on top of the
#   robot in forest_pillars_robot. Default: the world origin, on the ground.
#
# Run it in the PX4 terminal, after Gazebo is up (see docs/running_sim.md,
# "Flying without GPS"). It:
#   - runs PX4 in its OWN folder, build/px4_sitl_default/rootfs_nogps, so the
#     GPS-off settings never reach your normal parameters (PX4 saves every
#     `param set`, and by default always uses build/px4_sitl_default/rootfs);
#   - first copies your normal parameters there, so everything else (battery
#     drain, RC mode, calibration) matches your normal runs;
#   - turns GPS off and external vision on: EKF2_GPS_CTRL=0, EKF2_EV_CTRL=5
#     (horizontal position + velocity), EKF2_HGT_REF=0 (barometer height);
#   - spawns the camera drone, x500_lidar_cam_down (the only model with the
#     ground-truth feed sim_vio needs).
#
# Then start: the bridge with camera:=true vio:=true, sim_vio, pad_detector,
# patrol_lidar. Without sim_vio PX4 has no position and won't arm.
set -eu

WORLD=${1:-forest_pillars}
POSE=${2:-${PX4_GZ_MODEL_POSE:-}}
PX4_DIR=${PX4_DIR:-$HOME/dev/PX4-Autopilot}
BUILD=$PX4_DIR/build/px4_sitl_default
RUN_DIR=$BUILD/rootfs_nogps

[ -x "$BUILD/bin/px4" ] || { echo "PX4 not built: $BUILD/bin/px4"; exit 1; }
mkdir -p "$RUN_DIR"
cp "$BUILD/rootfs/gz_env.sh" "$RUN_DIR/"
if [ -f "$BUILD/rootfs/parameters.bson" ]; then      # start from your normal settings
    cp "$BUILD/rootfs/parameters.bson" "$RUN_DIR/parameters.bson"
    cp "$BUILD/rootfs/parameters.bson" "$RUN_DIR/parameters_backup.bson"
fi

# Gazebo and PX4 must not see ROS (its gz has no `sim` command).
strip() { echo "$1" | tr ':' '\n' | grep -v '^/opt/ros' | paste -sd: -; }
PATH=$(strip "$PATH"); LD_LIBRARY_PATH=$(strip "${LD_LIBRARY_PATH:-}")
PYTHONPATH=$(strip "${PYTHONPATH:-}"); export PATH LD_LIBRARY_PATH PYTHONPATH
unset GZ_CONFIG_PATH

cd "$RUN_DIR"
echo "PX4 without GPS in $RUN_DIR (world $WORLD${POSE:+, spawn at $POSE}); your normal parameters are only read."
if [ -n "$POSE" ]; then export PX4_GZ_MODEL_POSE="$POSE"; else unset PX4_GZ_MODEL_POSE; fi
# -w + the etc folder: without them PX4 ignores the current folder and uses rootfs/.
exec env PX4_PARAM_EKF2_GPS_CTRL=0 PX4_PARAM_EKF2_EV_CTRL=5 PX4_PARAM_EKF2_HGT_REF=0 \
    PX4_SYS_AUTOSTART=4013 PX4_SIM_MODEL=gz_x500_lidar_cam_down \
    PX4_GZ_STANDALONE=1 PX4_GZ_WORLD="$WORLD" \
    "$BUILD/bin/px4" -w "$RUN_DIR" "$BUILD/etc"
