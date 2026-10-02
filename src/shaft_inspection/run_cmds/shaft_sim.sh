#!/bin/bash
# Full vertical-shaft inspection simulation.
#   PX4 SITL (shaft world) + Micro XRCE agent + ROS 2 stack
# Ctrl-C stops everything.
set -u

WS=${WS:-$HOME/px4_ros2_ws}
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
LOGDIR=${LOGDIR:-$WS/log/shaft_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$LOGDIR"

cleanup() {
    echo; echo "stopping..."
    # Match executables by path, not by name: a bare "shaft_mission" pattern
    # also kills any shell or editor whose command line mentions it.
    pkill -f "ros2 launch shaft_inspection" 2>/dev/null
    pkill -f "lib/shaft_inspection/shaft_"  2>/dev/null
    pkill -f parameter_bridge    2>/dev/null
    pkill -f "px4_sitl_default/bin/px4" 2>/dev/null
    sleep 2
    pkill -f "gz sim"            2>/dev/null
    pkill -f MicroXRCEAgent      2>/dev/null
    pkill -f rviz2               2>/dev/null
    # QGroundControl deliberately NOT killed (see the QGC block below)
    sleep 1
    pkill -9 -f "px4_sitl_default/bin/px4" 2>/dev/null
    pkill -9 -f "gz sim"         2>/dev/null
    echo "logs in $LOGDIR"
}
trap cleanup EXIT INT TERM

# A Gazebo server left over from an earlier run is silently reused by PX4
# ("gazebo already running world: default"), which spawns the vehicle in the
# WRONG world while everything appears to start normally.  Clear them first.
echo "=== clearing stale sim processes ==="
pkill -9 -f "px4_sitl_default/bin/px4" 2>/dev/null
pkill -9 -f "gdb -q -batch"             2>/dev/null
pkill -9 -f "gz sim"                   2>/dev/null
pkill -9 -x MicroXRCEAgent             2>/dev/null
pkill -9 -f "lib/ros_gz_bridge/parameter_bridge" 2>/dev/null
pkill -9 -f "lib/shaft_inspection/shaft_" 2>/dev/null
sleep 2

echo "=== Micro XRCE-DDS agent ==="
MicroXRCEAgent udp4 -p 8888 > "$LOGDIR/agent.log" 2>&1 &
sleep 2

echo "=== PX4 SITL (shaft world, x500_shaft) ==="
if [ "${GDB:-0}" = "1" ]; then
    # Backtrace on abort lands in px4.log (see px4_shaft_gdb.sh).
    make -C "$PX4_DIR" px4_sitl_default > "$LOGDIR/build.log" 2>&1 || {
        echo "PX4 build failed - see $LOGDIR/build.log"; exit 1; }
    script -qf -c "$WS/run_cmds/px4_shaft_gdb.sh" "$LOGDIR/px4.log" > /dev/null 2>&1 &
else
    # Run under `script` to give PX4 a pty.  Without one, its pxh> prompt
    # repaints on every poll and the log grows to hundreds of MB in minutes.
    # -f flushes every write: without it the "Startup script returned" line
    # can sit in the buffer, the wait below times out, and the cleanup trap
    # kills a PX4 that started perfectly well.
    script -qf -c "cd '$PX4_DIR' && \
      HEADLESS=${HEADLESS:-1} \
      PX4_GZ_MODEL_POSE='2.20,0,0.06,0,0,0' \
      PX4_GZ_WORLD=vshaft \
      make px4_sitl gz_x500_shaft" "$LOGDIR/px4.log" > /dev/null 2>&1 &
fi

echo "waiting for PX4 to come up..."
for i in $(seq 1 120); do
    grep -q "Startup script returned successfully" "$LOGDIR/px4.log" 2>/dev/null && break
    sleep 2
done
if ! grep -q "Startup script returned successfully" "$LOGDIR/px4.log" 2>/dev/null; then
    echo "PX4 did not start — see $LOGDIR/px4.log"; tail -30 "$LOGDIR/px4.log"; exit 1
fi
echo "PX4 up."

# Optional Gazebo GUI:  GUI=1 ./shaft_sim.sh
if [ "${GUI:-0}" = "1" ]; then
    echo "=== Gazebo GUI ==="
    gz sim -g > "$LOGDIR/gz_gui.log" 2>&1 &
    sleep 3
fi

# Optional QGroundControl (manual flying, mode switching):  QGC=1 ./shaft_sim.sh
# QGroundControl is left running across relaunches: it reconnects to the new
# PX4 on its own, and killing it (especially with SIGKILL) loses unsaved
# settings such as joystick button mappings.
if [ "${QGC:-0}" = "1" ]; then
    if pgrep -f "QGroundControl" > /dev/null; then
        echo "=== QGroundControl already running (reusing) ==="
    else
        echo "=== QGroundControl ==="
        ( cd "$HOME/Downloads" && setsid ./QGroundControl-x86_64.AppImage ) > "$LOGDIR/qgc.log" 2>&1 &
    fi
fi

# Optional RViz:  RVIZ=1 ./shaft_sim.sh
if [ "${RVIZ:-0}" = "1" ]; then
    echo "=== RViz ==="
    ( set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"
      rviz2 -d "$WS/install/shaft_inspection/share/shaft_inspection/config/shaft.rviz" \
        --ros-args -p use_sim_time:=true ) > "$LOGDIR/rviz.log" 2>&1 &
fi

echo "=== ROS 2 shaft stack ==="
echo "    dashboard: http://localhost:8080  (read-only)"
# ROS setup scripts reference unset vars; -u would abort on them.
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u
ros2 launch shaft_inspection shaft_bringup.launch.py start_mode:=${MODE:-auto_launch} 2>&1 | tee "$LOGDIR/stack.log"
