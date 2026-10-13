#ifndef SYSTEMFAULTS_H
#define SYSTEMFAULTS_H

#include <stdint.h>
#include <stdbool.h>

#define SET_FAULT(mask, fault)((mask) |= (uint32_t)(fault))
#define CLEAR_FAULT(mask, fault)((mask) &= ~((uint32_t)(fault)))
#define HAS_FAULT(mask, fault)(((mask) & (uint32_t)(fault)) != 0U)
#define HAS_ANY_FAULT(mask)((mask) != FAULT_NONE)

#define GET_MOTOR_FAULTS(mask)((uint8_t)((mask) & 0x7FU))
#define GET_WIFI_FAULTS(mask)((uint8_t)(((mask) & 0x3F80U) >> 7))
#define GET_IR_FAULTS(mask)((uint8_t)(((mask) & 0x1FC000U) >> 14))
#define GET_US_FAULTS(mask)((uint8_t)(((mask) & 0x0FE00000U) >> 21))
#define GET_IMU_FAULTS(mask)((uint8_t)(((mask) & 0xF0000000U) >> 28))

extern volatile uint32_t active_fault_mask;


typedef enum{ // bit masking
    
    FAULT_NONE = 0x00000000U, // no active faults

    FAULT_MOTOR_LEFT_STALL = (1U << 0),     // Bit 0
    FAULT_MOTOR_RIGHT_STALL = (1U << 1),    // Bit 1
    FAULT_MOTOR_OVERCURRENT = (1U << 2),    // Bit 2
    FAULT_MOTOR_OVERVOLTAGE = (1U << 3),    // Bit 3
    FAULT_MOTOR_UNDERVOLTAGE = (1U << 4),   // Bit 4
    // reserved for any more motor faults (1U << 5)
    // reserved for any more motor faults (1U << 6)

    // for rayykkk
    FAULT_WIFI_DISCONNECTED = (1U << 7), // Bit 7
    FAULT_MQTT_DROPPED      = (1U << 8), // Bit 8
    FAULT_CMD_REJECTED      = (1U << 9), // Bit 9
    // <fault name> = (1U << 10),   // Bit 10
    // <fault name> = (1U << 11),   // Bit 11
    // <fault name> = (1U << 12),   // Bit 12
    // <fault name> = (1U << 13),   // Bit 13

    // for brownie
    // <fault name> = (1U << 14),    // Bit 14
    // <fault name> = (1U << 15),    // Bit 15
    // <fault name> = (1U << 16),    // Bit 16
    // <fault name> = (1U << 17),    // Bit 17
    // <fault name> = (1U << 18),    // Bit 18
    // <fault name> = (1U << 19),    // Bit 19
    // <fault name> = (1U << 20),    // Bit 20

    // for tim o tea
    // <fault name> = (1U << 21),    // Bit 21
    // <fault name> = (1U << 22),    // Bit 22
    // <fault name> = (1U << 23),    // Bit 23
    // <fault name> = (1U << 24),    // Bit 24
    // <fault name> = (1U << 25),    // Bit 25
    // <fault name> = (1U << 26),    // Bit 26
    // <fault name> = (1U << 27),    // Bit 27

    // for Fei
    // <fault name> = (1U << 28),    // Bit 28
    // <fault name> = (1U << 29),    // Bit 29
    // <fault name> = (1U << 30),    // Bit 30
    // <fault name> = (1U << 31),    // Bit 31
    // <fault name> = (1U << 32),    // Bit 32
    // <fault name> = (1U << 33),    // Bit 33
    // <fault name> = (1U << 34),    // Bit 34
} SystemFault_t;


#endif
