
Once youre done, don't forget to update the CMakeLists.txt to include your .c file.

Refer to systemFaults.h, yall can use 7 unique bits for your component faults (if any). Since the pico w is a 32-bit architecture, we have 32 bits to play around with.
But i think 7 bits per person shld be good enuf.

Also refer to safetyMonitor.c, its where we dump all our fault checks here since it makes debugging easier.



Miscellaneous connections:
Piezo Buzzer - GP22



LEFT MOTOR connections:
| *Sensor/Function* | *Connected To Pin/Terminal* | *Cable Colour* |
| Encoder GND       | GROUND                      | WHITE          |
| Left Encoder A    | GP14                        | RED            |
| Right Encoder B   | GP15                        | GREEN          |
| Encoder Power     | 3.3V                        | BLUE           |
| Motor Power B     | M1B                         | YELLOW         |
| Motor Power A     | M1A                         | BLACK          |

RIGHT MOTOR connections:
| *Sensor/Function* | *Connected To Pin/Terminal* | *Cable Colour* |
| Encoder GND       | GROUND                      | WHITE->BLACK   |
| Left Encoder A    | GP2                         | RED->WHITE     |
| Right Encoder B   | GP3                         | GREEN->YELLOW  |
| Encoder Power     | 3.3V                        | BLUE->RED      |
| Motor Power B     | M2B                         | YELLOW         |
| Motor Power A     | M2A                         | BLACK          |

HC-SR04 ULTRASONIC SENSOR connections(GROVE1):
| *Sensor/Function* | *Connected To Pin/Terminal* | *Cable Colour* |
| VCC               | 3.3V                        | RED            |
| Trigger Signal    | GP0                         | WHITE          |
| Echo Signal       | GP1                         | YELLOW         |
| GND               | GROUND                      | BLACK          |

GY-511 3-AXIS ACCELEROMETER AND MAGNETOMETER connections(GROVE6):
| *Sensor/Function* | *Connected To Pin/Terminal* | *Cable Colour* |
| 3.3V              | 3.3V                        | RED            |
| GND               | GROUND                      | BLACK          |
| Serial Clock      | GP26                        | WHITE          |
| Serial Data       | GP27                        | YELLOW         |

TCRT5000 REFLECTIVE OPTICAL SENSOR connections(GROVE7, left):
| *Sensor/Function* | *Connected To Pin/Terminal* | *Cable Colour* |
| VCC               | 3.3V                        | RED            |
| GND               | GROUND                      | BLACK          |
| Digital Output    | GP7                         | WHITE          |
| Analog Output     | GP28                        | YELLOW         |

TCRT5000 REFLECTIVE OPTICAL SENSOR connections(GROVE3, right):
| *Sensor/Function* | *Connected To Pin/Terminal* | *Cable Colour* |
| VCC               | 3.3V                        | RED            |
| GND               | GROUND                      | BLACK          |
| Digital Output    | GP4                         | WHITE          |
| Analog Output     | GP5                         | YELLOW         |

TCRT5000 REFLECTIVE OPTICAL SENSOR connections(GROVE4, edge):
| *Sensor/Function* | *Connected To Pin/Terminal* | *Cable Colour* |
| VCC               | 3.3V                        | RED            |
| GND               | GROUND                      | BLACK          |
| Digital Output    | GP16                        | WHITE          |
| Analog Output     | GP17                        | YELLOW         |

SERVO connections:
| *Sensor/Function* | *Connected To Pin/Terminal* | *Cable Colour* |
| GND               | GROUND                      | BROWN          |
| VCC               | 3.3V                        | RED            |
| PWM Control Signal| GP12                        | ORANGE         |


