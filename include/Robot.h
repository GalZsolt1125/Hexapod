#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "RobotGeometry.h"
#include "RobotTypes.h"
#include "ServoMotor.h"
#include "ServoConfig.h"
#include "RobotController.h"

class Robot {
private:
    Adafruit_PWMServoDriver pwmLeft;
    Adafruit_PWMServoDriver pwmRight;

    ServoMotor coxa[6];
    ServoMotor femur[6];
    ServoMotor tibia[6];

    RobotController* controller;

public:
    Robot();

    void begin();

    bool setLegPosition(
        LegID leg,
        const Point3D& position
    );

    bool setAllLegsPosition(
        const Point3D positions[6]
    );

    Point3D getLegPosition(LegID leg) const;
};

#endif