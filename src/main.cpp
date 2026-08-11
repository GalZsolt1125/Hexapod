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
}

void loop() {
    float timeSec = (float)millis() / 1000.0f;
    float cycleTime = fmod(timeSec, 10.0f);

    if (cycleTime < 5.0f) {

        if (!robot.isMoving()) {
            robot.setMotion(0.0f, 1.0f, 0.0f);
            robot.start();
        }

    } else {

        if (robot.isMoving()) {
            robot.stop();
        }
    }

    robot.update();
    delay(20);
}