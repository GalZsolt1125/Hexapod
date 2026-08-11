#include <Arduino.h>

#include "Robot.h"
#include "Gait.h"

Robot robot;
Gait gait;

Point3D homeBodyPositions[6];

float bodyHeight = -20.0f;

void setup() {

    Serial.begin(115200);

    robot.begin();

    for (int i = 0; i < 6; i++) {

        Point3D homeLocal = {
            95.255f,
            0.0f,
            -146.306f
        };

        homeBodyPositions[i] =
            localToBody(
                static_cast<LegID>(i),
                homeLocal
            );
    }

    robot.setAllLegsPosition(
        homeBodyPositions
    );

    robot.setBodyHeight(
        bodyHeight
    );

    for (int i = 0; i < 6; i++) {

        homeBodyPositions[i] =
            robot.getLegPosition(
                static_cast<LegID>(i)
            );
    }

    gait.setHomePositions(
        homeBodyPositions
    );

    gait.setType(
        GaitType::Tripod
    );

    gait.setStepLength(
        80.0f
    );

    gait.setStepHeight(
        40.0f
    );

    gait.setStepPeriod(
        0.8f
    );
}

void loop() {

    gait.setMotion(
        0.0f,
        1.0f,
        8.0f
    );

    float period = 0.8f;

    float phase =
        fmod(
            (float)millis()
            /
            (period * 1000.0f),
            1.0f
        );

    gait.update(
        robot,
        phase
    );

    delay(20);
}