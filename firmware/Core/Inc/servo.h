#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "main.h"
#include <stdint.h>

/*
 * Servo calibration
 */

#define SERVO0_PULSE_NEG90   95
#define SERVO0_PULSE_0       301
#define SERVO0_PULSE_POS90   492

#define SERVO1_PULSE_NEG90   105
#define SERVO1_PULSE_0       310
#define SERVO1_PULSE_POS90   500

#define SERVO2_PULSE_NEG90   102
#define SERVO2_PULSE_0       308
#define SERVO2_PULSE_POS90   505

#define SERVO3_PULSE_NEG90   100
#define SERVO3_PULSE_0       312
#define SERVO3_PULSE_POS90   510

#define SERVO4_PULSE_NEG90   100
#define SERVO4_PULSE_0       305
#define SERVO4_PULSE_POS90   517

#define SERVO5_PULSE_NEG90   106
#define SERVO5_PULSE_0       312
#define SERVO5_PULSE_POS90   525

#define SERVO6_PULSE_NEG90   99
#define SERVO6_PULSE_0       308
#define SERVO6_PULSE_POS90   512

#define SERVO7_PULSE_NEG90   102
#define SERVO7_PULSE_0       305
#define SERVO7_PULSE_POS90   512

#define SERVO8_PULSE_NEG90   102
#define SERVO8_PULSE_0       308
#define SERVO8_PULSE_POS90   512

#define SERVO9_PULSE_NEG90   105
#define SERVO9_PULSE_0       308
#define SERVO9_PULSE_POS90   515

#define SERVO10_PULSE_NEG90   100
#define SERVO10_PULSE_0       308
#define SERVO10_PULSE_POS90   520

#define SERVO11_PULSE_NEG90   100
#define SERVO11_PULSE_0       306
#define SERVO11_PULSE_POS90   503

#define SERVO12_PULSE_NEG90   105
#define SERVO12_PULSE_0       305
#define SERVO12_PULSE_POS90   500

#define SERVO13_PULSE_NEG90   98
#define SERVO13_PULSE_0       308
#define SERVO13_PULSE_POS90   508

#define SERVO14_PULSE_NEG90   97
#define SERVO14_PULSE_0       316
#define SERVO14_PULSE_POS90   516

//------------SERVOS CALIBRATED UNTIL HERE (SERVOS 0~11, LEGS A~D)------------------------------

#define SERVO15_PULSE_NEG90   97
#define SERVO15_PULSE_0       303
#define SERVO15_PULSE_POS90   500

#define SERVO16_PULSE_NEG90   97
#define SERVO16_PULSE_0       303
#define SERVO16_PULSE_POS90   507

#define SERVO17_PULSE_NEG90   102
#define SERVO17_PULSE_0       308
#define SERVO17_PULSE_POS90   516


void Servo_Init(void);

void Servo_SetAngle(uint8_t servo,
                    float angle);


#endif /* INC_SERVO_H_ */
