#include "InverseKinematics.h"
#include <Arduino.h>

void printResult(const IKResult& result) {
    if (!result.valid) {
        Serial.println("IK: INVALID / UNREACHABLE");
        return;
    }

    Serial.println("IK: VALID");

    Serial.print("Coxa:  ");
    Serial.println(result.angles.coxa, 4);

    Serial.print("Femur: ");
    Serial.println(result.angles.femur, 4);

    Serial.print("Tibia: ");
    Serial.println(result.angles.tibia, 4);
}

void setup() {
    Serial.begin(115200);

    delay(1000);

    Point3D footLocal = {
        88.038f + 48.217f,
        0.0f,
        -108.306f
    };

    Serial.println();
    Serial.println("=== IK TEST ===");

    Serial.print("X: ");
    Serial.println(footLocal.x, 4);

    Serial.print("Y: ");
    Serial.println(footLocal.y, 4);

    Serial.print("Z: ");
    Serial.println(footLocal.z, 4);

    IKResult result = calculateIK(footLocal);

    printResult(result);
}

void loop() {
}