#!/usr/bin/env bash
# Demo: fly patrol_lidar in the simulator with live RViz, saving nothing.
#
#   scripts/demo_patrol_lidar.sh [world]   start (default world: forest_pillars)
#   scripts/demo_patrol_lidar.sh stop      stop everything and close the windows
#
# Opens one window each for: DDS agent, Gazebo, PX4, lidar bridge,
# QGroundControl, RViz and patrol_lidar. patrol_lidar runs with map_dir:=''
# so no map or objects CSV is written, and no bag is recorded. (PX4 still keeps
# its own flight log, as it does on every SITL run.)
#
# Worlds: forest_pillars, lidar_test (both need the one-time link into PX4's
# worlds folder, see docs/running_sim.md), forest (PX4's own), default.
#
# Paths can be overridden: PX4_DIR, PX4_WS, QGC (QGroundControl AppImage),
# ALT (patrol altitude, m). Build the workspace first (colcon build).
set -u

REPO=$(cd "$(dirname "$(realpath "$0")")/.." && pwd)
SELF=$(realpath "$0")
PX4_DIR=${PX4_DIR:-$HOME/dev/PX4-Autopilot}
PX4_WS=${PX4_WS:-$HOME/dev/px4_ws}
QGC=${QGC:-$HOME/dev/tools/QGroundControl-x86_64.AppImage}
ALT=${ALT:-2.0}
STATE=${XDG_RUNTIME_DIR:-/tmp}/las_demo_$(id -u)   # window shell PIDs, for stop
SHELLS=$STATE/shells

ros_env() {
    set +u      # ROS setup files read unset variables
    source /opt/ros/jazzy/setup.bash
    source "$PX4_WS/install/setup.bash"
    source "$REPO/install/setup.bash"
    set -u
}

# Gazebo and PX4 must not see ROS: ROS Jazzy's own `gz` has no `sim` command
# and its GZ_CONFIG_PATH breaks Gazebo ("Timed out waiting for Gazebo world").
no_ros_env() {
    strip() { echo "$1" | tr ':' '\n' | grep -v '^/opt/ros' | paste -sd: -; }
    PATH=$(strip "$PATH")
    LD_LIBRARY_PATH=$(strip "${LD_LIBRARY_PATH:-}")
    PYTHONPATH=$(strip "${PYTHONPATH:-}")
    export PATH LD_LIBRARY_PATH PYTHONPATH
    unset GZ_CONFIG_PATH
}

# Open a terminal window running "$SELF _run <what> <world>". The window stays
# open after the program exits so errors stay readable; stop closes it.
window() {
    gnome-terminal --title="demo: $1" -- bash -c "\"$SELF\" _run $2 $3; exec bash"
}

wait_for() {   # wait_for <description> <seconds> <command...>
    local what=$1 secs=$2; shift 2
    printf '  waiting for %s...' "$what"
    for _ in $(seq "$secs"); do
        if "$@" >/dev/null 2>&1; then echo ' ok'; return 0; fi
        sleep 1
    done
    echo " timed out after ${secs}s (check its window)"; return 1
}

gz_world_up()  { env -u GZ_CONFIG_PATH PATH=/usr/bin:/bin gz topic -l | grep -q "/world/$1/clock"; }
px4_up()       { (ros_env; ros2 topic list | grep -q fmu/out/vehicle_status); }
scan_up()      { (ros_env; timeout 5 ros2 topic echo --once /scan sensor_msgs/msg/LaserScan --field header.frame_id | grep -q .); }
gcs_up()       { (ros_env; ros2 topic echo --once --qos-reliability best_effort /fmu/out/failsafe_flags | grep -q 'gcs_connection_lost: false'); }

# ---------------------------------------------------------------- per window
run() {   # run <what> <world>; runs inside a demo window
    echo $PPID >> "$SHELLS"            # this window's shell, so stop can close it
    local world=$2
    case $1 in
        agent)   MicroXRCEAgent udp4 -p 8888 ;;
        gazebo)  no_ros_env; cd "$PX4_DIR" &&
                 python3 Tools/simulation/gz/simulation-gazebo --world "$world" \
                     --model_store "$PX4_DIR/Tools/simulation/gz" ;;
        px4)     no_ros_env; cd "$PX4_DIR" &&
                 PX4_GZ_STANDALONE=1 PX4_GZ_WORLD="$world" make px4_sitl gz_x500_lidar_2d ;;
        bridge)  ros_env; ros2 launch las_offboard lidar_bridge.launch.py world:="$world" ;;
        qgc)     "$QGC" ;;
        rviz)    ros_env; rviz2 -d "$(ros2 pkg prefix las_offboard)/share/las_offboard/rviz/patrol_lidar.rviz" ;;
        patrol)  ros_env; ros2 run las_offboard patrol_lidar --ros-args \
                     -p altitude_m:="$ALT" -p "map_dir:=''" ;;
    esac
}

# ---------------------------------------------------------------- stop
stop() {
    # patrol_lidar first (Ctrl-C), then everything else; each window's shell
    # leads its own process group, so signalling the group reaches its programs.
    pkill -INT -f 'lib/las_offboard/patrol_lidar' 2>/dev/null && sleep 2
    if [ -f "$SHELLS" ]; then
        local pids; pids=$(sort -u "$SHELLS")
        for p in $pids; do kill -INT -- "-$p" 2>/dev/null; done
        sleep 4
        for p in $pids; do kill -KILL -- "-$p" 2>/dev/null; kill -KILL "$p" 2>/dev/null; done
        rm -f "$SHELLS"
    fi
    # Anything that escaped its group (Gazebo's server/GUI, PX4, the QGC AppImage
    # launcher) on a second pass.
    pkill -KILL -f 'gz sim|px4_sitl_default/bin/px4|MicroXRCEAgent|parameter_bridge' 2>/dev/null
    [ -n "$QGC" ] && pkill -KILL -f "$(basename "$QGC")" 2>/dev/null
    echo "Demo stopped."
}

# ---------------------------------------------------------------- start
start() {
    local world=${1:-forest_pillars}
    if pgrep -f 'gz sim|px4_sitl_default/bin/px4|MicroXRCEAgent' >/dev/null; then
        echo "A simulator is already running. Stop it first: $0 stop"; exit 1
    fi
    if [ ! -f "$PX4_DIR/Tools/simulation/gz/worlds/$world.sdf" ]; then
        echo "World '$world' not found in $PX4_DIR/Tools/simulation/gz/worlds."
        echo "Link it in first (see docs/running_sim.md)."; exit 1
    fi
    if [ ! -f "$REPO/install/setup.bash" ]; then
        echo "Workspace not built. Run colcon build in $REPO first."; exit 1
    fi
    mkdir -p "$STATE"; : > "$SHELLS"

    echo "Starting patrol_lidar demo in '$world' (nothing will be saved)"
    window "DDS agent" agent "$world"
    window "Gazebo" gazebo "$world"
    wait_for "Gazebo world" 120 gz_world_up "$world" || exit 1
    window "PX4" px4 "$world"
    wait_for "PX4" 180 px4_up || exit 1
    window "Lidar bridge" bridge "$world"
    window "QGroundControl" qgc "$world"
    wait_for "lidar on /scan" 60 scan_up || exit 1
    wait_for "QGroundControl link" 60 gcs_up || exit 1
    window "RViz" rviz "$world"
    sleep 2
    window "patrol_lidar" patrol "$world"
    echo "Flying. Watch RViz. Stop with: $0 stop"
    echo "If patrol_lidar says 'Could not arm', PX4 wants stick input: turn on"
    echo "QGC's virtual joystick, or in the sim run 'param set COM_RC_IN_MODE 4' at pxh>."
}

case ${1:-} in
    stop)        stop ;;
    _run)        run "$2" "$3" ;;
    -h|--help)   sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//' ;;
    *)           start "${1:-}" ;;
esac
