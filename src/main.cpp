#include <Arduino.h>

#include "Robot.h"
#include "RobotUart.h"

Robot robot;

RobotUart robotUart(
    Serial,
    robot
);

void setup()
{
    Serial.begin(115200);

    robot.begin();
    robot.setGaitType(GaitType::Tripod);
    robot.setStepLength(70.0f);
    robot.setStepHeight(35.0f);
    robot.setStepPeriod(0.8f);
    
    robot.setBodyOffset(
        0.0f,
        0.0f,
        -20.0f
    );

    robot.calibration();

    robotUart.begin();
}

void loop()
{
    robot.update();

    robotUart.update();
}