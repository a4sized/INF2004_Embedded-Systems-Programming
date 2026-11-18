/**
 * @file    s5_scan.h
 * @brief   Subsystem 5, stages 2 to 4: fine scan an obstacle and decide.
 */
#ifndef S5_SCAN_H
#define S5_SCAN_H

#include "s5_profile.h"
#include "s5_avoid.h"

/*
 * Fine scan the obstacle ahead, publish its profile into *p_profile and
 * decide how to get past it (s5_avoid_plan). The car is held still the
 * whole time (speed cap 0), because the sensor is not watching the path
 * while it looks sideways. The servo faces ahead again afterwards. Call
 * only from the scanning task.
 */
s5_bypass_t s5_scan_obstacle (s5_profile_t * const p_profile);

#endif /* S5_SCAN_H */
