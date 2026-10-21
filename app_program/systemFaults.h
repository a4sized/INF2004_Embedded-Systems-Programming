#ifndef SYSTEMFAULTS_H
#define SYSTEMFAULTS_H

#include <tk/tkernel.h>

// global event flag
IMPORT ID fault_flag; 


#define FAULT_ESTOP      (1U << 0)  // Emergency stop
#define FAULT_MOTOR      (1U << 1)  // Motor fault
#define FAULT_IR_SENS    (1U << 2)  // IR sensor fault
#define FAULT_US_SENS    (1U << 3)  // Ultrasonic fault
#define FAULT_IMU_SENS   (1U << 4)  // IMU fault
#define FAULT_WIFI       (1U << 5)  // Wifi fault

// motor must stop, requires manual clearance
#define TIER1_FATAL_MASK  (FAULT_ESTOP | FAULT_MOTOR | FAULT_IR_SENS | FAULT_IMU_SENS)

// motor pauses and resumes once flag is cleared
#define TIER2_HAZARD_MASK (FAULT_WIFI | FAULT_US_SENS)

#define SET_FAULT(fault)   tk_set_flg(fault_flag, (fault))
#define CLEAR_FAULT(fault) tk_clr_flg(fault_flag, ~(fault))

#endif 