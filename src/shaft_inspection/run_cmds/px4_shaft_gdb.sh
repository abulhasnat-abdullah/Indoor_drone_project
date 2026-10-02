#!/bin/bash
# PX4 SITL for the shaft world, run directly under gdb so an abort leaves a
# backtrace.  Called by shaft_sim.sh when GDB=1 (make's run target cannot be
# wrapped by gdb).
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
cd "$PX4_DIR/build/px4_sitl_default/src/modules/simulation/gz_bridge" || exit 1
export HEADLESS=${HEADLESS:-1}
export PX4_GZ_MODEL_POSE=${PX4_GZ_MODEL_POSE:-2.20,0,0.06,0,0,0}
export PX4_GZ_WORLD=vshaft
export PX4_SIM_MODEL=gz_x500_shaft
export GZ_IP=127.0.0.1
# SIGCONT is used internally by the hrt/lockstep threads; gdb must not stop on it.
exec gdb -q -batch \
  -ex "set pagination off" \
  -ex "handle SIGPIPE nostop noprint pass" \
  -ex "handle SIGCONT nostop noprint pass" \
  -ex run \
  -ex 'printf "\n===== PX4 ABORTED - BACKTRACE =====\n"' \
  -ex "thread apply all bt" \
  -ex 'printf "\n===== END BACKTRACE =====\n"' \
  --args "$PX4_DIR/build/px4_sitl_default/bin/px4"
