#ifndef ROBOT_SERVOS_H
#define ROBOT_SERVOS_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

#include "ServoMotor.h"
#include "ServoConfig.h"

extern Adafruit_PWMServoDriver pwmLeft;
extern Adafruit_PWMServoDriver pwmRight;

namespace RobotServos {

ServoMotor coxaLF(
    &pwmLeft, 0, 150, 550,
    5.0f, true,
    -90.0f, 90.0f
);

ServoMotor femurLF(
    &pwmLeft, 1, 150, 550,
    -4.0f, true,
    -90.0f, 90.0f
);

ServoMotor tibiaLF(
    &pwmLeft, 2, 150, 550,
    0.0f, true,
    -90.0f, 90.0f
);

ServoMotor coxaLM(
    &pwmLeft, 4, 150, 550,
    6.0f, true,
    -90.0f, 90.0f
);

ServoMotor femurLM(
    &pwmLeft, 5, 150, 550,
    2.0f, true,
    -90.0f, 90.0f
);

ServoMotor tibiaLM(
    &pwmLeft, 6, 150, 550,
    -5.0f, true,
    -90.0f, 90.0f
);

ServoMotor coxaLR(
    &pwmLeft, 8, 150, 550,
    8.0f, true,
    -90.0f, 90.0f
);

ServoMotor femurLR(
    &pwmLeft, 9, 150, 550,
    -5.0f, true,
    -90.0f, 90.0f
);

ServoMotor tibiaLR(
    &pwmLeft, 10, 150, 550,
    0.0f, true,
    -90.0f, 90.0f
);


ServoMotor coxaRF(
    &pwmRight, 15, 150, 550,
    6.0f, false,
    -90.0f, 90.0f
);

ServoMotor femurRF(
    &pwmRight, 14, 150, 550,
    -5.0f, false,
    -90.0f, 90.0f
);

ServoMotor tibiaRF(
    &pwmRight, 13, 150, 550,
    0.0f, false,
    -90.0f, 90.0f
);

ServoMotor coxaRM(
    &pwmRight, 11, 150, 550,
    0.0f, false,
    -90.0f, 90.0f
);

ServoMotor femurRM(
    &pwmRight, 10, 150, 550,
    2.0f, false,
    -90.0f, 90.0f
);

ServoMotor tibiaRM(
    &pwmRight, 9, 150, 550,
    -7.0f, false,
    -90.0f, 90.0f
);

ServoMotor coxaRR(
    &pwmRight, 7, 150, 550,
    4.0f, false,
    -90.0f, 90.0f
);

ServoMotor femurRR(
    &pwmRight, 6, 150, 550,
    -3.0f, false,
    -90.0f, 90.0f
);

ServoMotor tibiaRR(
    &pwmRight, 5, 150, 550,
    0.0f, false,
    -90.0f, 90.0f
);

}

#endif