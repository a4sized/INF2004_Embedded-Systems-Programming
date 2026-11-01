
Once youre done, don't forget to update the CMakeLists.txt to include your .c file.

**Unresolved pin conflict (1 November 2026):** This table assigns GP4 to
the right motor encoder and GP2 to the right line sensor. The current
Buddy 3 IR firmware and the user's last confirmed line-sensor setup use
GP4 for the right line sensor (GP7 left, GP6 barcode). GP4 cannot serve
both devices. The IR-branch's earlier encoder table used GP14/GP15 and
GP2/GP3 instead. Confirm the car wiring with the team and update both
subsystems before a combined motor/IR test; this merge does not settle
the allocation.



Miscellaneous connections:
Piezo Buzzer - GP22



LEFT MOTOR connections: **UPDATED**
| *Sensor/Function* | *Connected To Pin/Terminal* | *Cable Colour*   |
| Encoder GND       | GROUND                      | **BLUE->BLACK**  |
| Left Encoder B    | **GP16**                    | **GREEN->WHITE** |
| Right Encoder A   | **GP17**                    | **RED->YELLOW**  |
| Encoder Power     | 3.3V                        | **WHITE->RED**   |
| Motor Power B     | M1B                         | YELLOW           |
| Motor Power A     | M1A                         | BLACK            |

RIGHT MOTOR connections: **UPDATED**
| *Sensor/Function* | *Connected To Pin/Terminal* | *Cable Colour*   |
| Encoder GND       | GROUND                      | **BLUE->BLACK**  |
| Left Encoder B    | **GP4**                     | **GREEN->WHITE** |
| Right Encoder A   | **GP5**                     | **RED->YELLOW**  |
| Encoder Power     | 3.3V                        | **WHITE->RED**   |
| Motor Power B     | M2B                         | YELLOW           |
| Motor Power A     | M2A                         | BLACK            |

HC-SR04 ULTRASONIC SENSOR connections(GROVE1): **NO CHANGE**
| *Sensor/Function* | *Connected To Pin/Terminal* | *Cable Colour* |
| VCC               | 3.3V                        | RED            |
| Trigger Signal    | GP0                         | WHITE          |
| Echo Signal       | GP1                         | YELLOW         |
| GND               | GROUND                      | BLACK          |

GY-511 3-AXIS ACCELEROMETER AND MAGNETOMETER connections(GROVE6): **NO CHANGE**
| *Sensor/Function* | *Connected To Pin/Terminal* | *Cable Colour* |
| 3.3V              | 3.3V                        | RED            |
| GND               | GROUND                      | BLACK          |
| Serial Clock      | GP27                        | WHITE          |
| Serial Data       | GP26                        | YELLOW         |

TCRT5000 REFLECTIVE OPTICAL SENSOR connections(GROVE7, left): **NO CHANGE**
| *Sensor/Function* | *Connected To Pin/Terminal* | *Cable Colour* |
| VCC               | 3.3V                        | RED            |
| GND               | GROUND                      | BLACK          |
| Digital Output    | GP7                         | WHITE          |
| Analog Output     | GP28                        | YELLOW         |

TCRT5000 REFLECTIVE OPTICAL SENSOR connections(GROVE2, right): **UPDATED**
| *Sensor/Function* | *Connected To Pin/Terminal* | *Cable Colour* |
| VCC               | 3.3V                        | RED            |
| GND               | GROUND                      | BLACK          |
| Digital Output    | **GP2**                     | WHITE          |
| Analog Output     | **GP3**                     | YELLOW         |

TCRT5000 REFLECTIVE OPTICAL SENSOR connections(GROVE4, edge): **UPDATED**
| *Sensor/Function* | *Connected To Pin/Terminal* | *Cable Colour* |
| VCC               | 3.3V                        | RED            |
| GND               | GROUND                      | BLACK          |
| Digital Output    | **GP6**                     | WHITE          |
| Analog Output     | **GP26** DO NOT USE         | YELLOW         |

SERVO connections: **NO CHANGE**
| *Sensor/Function* | *Connected To Pin/Terminal* | *Cable Colour* |
| GND               | GROUND                      | BROWN          |
| VCC               | 3.3V                        | RED            |
| PWM Control Signal| GP12                        | ORANGE         |

