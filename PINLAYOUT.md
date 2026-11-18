
Once youre done, don't forget to update the CMakeLists.txt to include your .c file.

**Current target allocation (1 November 2026):** Buddy 3 firmware follows
this table: GP7 left line, GP2 right line and GP6 barcode. Buddy 2 uses
GP4 for right encoder B. Earlier GP4 right-line readings are historical.
Rewire and verify GP2 before a combined motor/IR test. This table places
the GP6 sensor on Grove 4; the user's earlier physical wiring report said
Grove 5, so the connector position still needs checking on the robot.



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
