#include <Arduino.h>
#include "Robot.h"

Robot robot;

void setup() {
    Serial.begin(115200);

    robot.begin();

    robot.setGaitType(GaitType::Tripod);
    robot.setStepLength(70.0f);
    robot.setStepHeight(35.0f);
    robot.setStepPeriod(0.6f);

    robot.setBodyOffset(0.0f, 0.0f, -25.0f);
}

void loop() {
    robot.setMotion(0.0f, 0.0f, 0.0f);

    robot.update();

    delay(20);
}