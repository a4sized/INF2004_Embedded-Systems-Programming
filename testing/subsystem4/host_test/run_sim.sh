#!/bin/sh
# Runs subsystem 4's REAL code on a PC (needs gcc and pthreads).
# Must stay inside the repo (testing/subsystem4/host_test): it compiles the
# firmware in app_program/subsystem4 against the real header of subsystem 1.
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

echo "=== packing: sizes, limits, wrap-round, the accumulator (test_pack.c)"
gcc -std=gnu11 -O1 -Wall -Wextra -Wno-unused-parameter -DS4_HOST_SIM -Isim -I$S -I$APP -I../../../device/include test_pack.c -o /tmp/s4_pack -lm
/tmp/s4_pack
echo
build s4_sim
echo "=== scenario 1: full drive (13 s): still, accelerate, a hard knock (NOT a fault any more), hump, turn. Distance from SUBSYSTEM 2's get_distance_traveled_mm()"
/tmp/s4_sim drive
echo
echo "=== scenario 2: the same drive WITHOUT encoders (S4_USE_ENCODER_DISTANCE 0): fixed-speed assumption"
build s4_sim_fallback -DS4_USE_ENCODER_DISTANCE=0
/tmp/s4_sim_fallback drive
echo
echo "=== scenario 2b: the encoder counters are zeroed IN THE MIDDLE of the hump (motor_reset_ticks): the hump must still be measured"
/tmp/s4_sim reset
echo
echo "=== scenario 3: picked up, carried, put down: the motors are halted at once and released after it has rested (automatic release)"
/tmp/s4_sim lift
echo
echo "=== scenario 3a: the same with S4_LIFT_LATCH = 1: stays latched until s4_clear_lifted(), accepted only while resting level"
build s4_sim_latch -DS4_LIFT_LATCH=1
/tmp/s4_sim_latch lift
echo
echo "=== scenario 3b: no ghost fault.  Put down and released after 0.6 s; Subsystem 1's LED task is run at 6 different phases"
build s4_sim_ghost -DS4_PUTDOWN_MS=600
for phase in 0 40 80 120 160 200; do LED_PHASE_MS=$phase /tmp/s4_sim_ghost ghost || exit 1; done
echo
echo "=== scenario 3c: tipped onto its side, left lying, then set upright"
/tmp/s4_sim overturn
echo
echo "=== scenario 3d: turned upside down"
/tmp/s4_sim flip
echo
echo "=== scenario 3e: FALSE ALARM test: rough ground (0.12 g shaking) with a knock, a vertical bump and a sideways shove"
/tmp/s4_sim rough
echo
echo "=== scenario 3f: FALSE ALARM test: a 30 degree slope"
/tmp/s4_sim ramp
echo
echo "=== scenario 4: sensor unplugged"
/tmp/s4_sim nosensor
echo
echo "=== scenario 5: sensor plugged in after 3 s (cable fixed while running)"
/tmp/s4_sim plugin
echo
echo "=== scenario 6: board mounted the other way up, mapping not changed"
/tmp/s4_sim wrongmount
