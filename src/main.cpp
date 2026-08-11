#include <Arduino.h>
#include "Robot.h"

Robot robot;

void setup() {
    Serial.begin(115200);

    robot.begin();

    robot.setGaitType(GaitType::Tripod);
    robot.setStepLength(70.0f);
    robot.setStepHeight(35.0f);
    robot.setStepPeriod(0.8f);

    robot.setBodyOffset(0.0f, 0.0f, -20.0f);
    robot.calibration();
}

void loop() {
    delay(20);
}