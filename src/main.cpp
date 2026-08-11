#include <Arduino.h>
#include <math.h>

#include "Robot.h"

Robot robot;

Point3D homeBodyPositions[6];

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
}

void loop() {

    float timeSec = (float)millis() / 1000.0f;
    float cycleTime = fmod(timeSec, 12.0f);

    if (cycleTime < 4.0f) {

        float phase = (cycleTime / 4.0f) * 2.0f * MY_PI;
        float xOffset = 30.0f * sin(phase);

        robot.setBodyOffset(xOffset, 0.0f, -20.0f);

    } else if (cycleTime < 8.0f) {

        float phase = ((cycleTime - 4.0f) / 4.0f) * 2.0f * MY_PI;
        float yOffset = 30.0f * sin(phase);

        robot.setBodyOffset(0.0f, yOffset, -20.0f);

    } else {

        float phase = ((cycleTime - 8.0f) / 4.0f) * 2.0f * MY_PI;
        float zOffset = -20.0f + 15.0f * sin(phase);

        robot.setBodyOffset(0.0f, 0.0f, zOffset);
    }

    delay(20);
}