#!/bin/sh
# Runs subsystem 4's REAL code on a PC (needs gcc and pthreads).
# Must stay inside the repo (testing/subsystem4/host_test): it compiles the
# firmware in app_program/subsystem4 against the real headers of subsystems 1 and 2.
set -e
cd "$(dirname "$0")"
APP=../../../app_program
S=$APP/subsystem4
SRC="sim/sim_kernel.c $S/subsystem4.c $S/lsm303d.c $S/task_imu.c $S/task_report.c"

build () {   # $1 = output name, then extra compiler flags
    out=$1; shift
    gcc -std=gnu11 -g -O1 -Wall -Wextra -Wno-unused-parameter -DS4_HOST_SIM -DAPP_TEST=0 \
        "$@" -Isim -I$S -I$APP -I../../../device/include $SRC -o /tmp/$out -lm -lpthread
}

build s4_sim
echo "=== scenario 1: full drive (13 s): still, accelerate, knock, hump, turn. Distance = fixed-speed assumption"
/tmp/s4_sim drive
echo
echo "=== scenario 2: the same drive, distance from SUBSYSTEM 2's encoder getters (S4_CM_PER_TICK set)"
build s4_sim_ticks -DS4_CM_PER_TICK='(0.05f)'
/tmp/s4_sim_ticks drive
echo
echo "=== scenario 3: collision stays latched (S4_COLLISION_HOLD_MS = 0) until s4_clear_collision()"
build s4_sim_latch -DS4_COLLISION_HOLD_MS=0
/tmp/s4_sim_latch latch
echo
echo "=== scenario 4: sensor unplugged"
/tmp/s4_sim nosensor
echo
echo "=== scenario 5: sensor plugged in after 3 s (cable fixed while running)"
/tmp/s4_sim plugin
echo
echo "=== scenario 6: board mounted the other way up, mapping not changed"
/tmp/s4_sim wrongmount
