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
#define FAULT_CMD_REJECTED (1U << 6) // MQTT invalid command
#define FAULT_IR_NAV     (1U << 7)  // Junction turn could not reacquire line

// motor must stop, requires manual clearance
#define TIER1_FATAL_MASK  (FAULT_ESTOP | FAULT_MOTOR | FAULT_IR_SENS | FAULT_IMU_SENS | FAULT_IR_NAV)

// motor pauses for a local sensing hazard; Wi-Fi loss stays diagnostic
#define TIER2_HAZARD_MASK (FAULT_US_SENS)


#endif 
