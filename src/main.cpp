#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "RobotGeometry.h"
#include "RobotTypes.h"
#include "InverseKinematics.h"
#include "ServoMotor.h"
#include "ServoConfig.h"

Adafruit_PWMServoDriver pwmLeft(0x40);
Adafruit_PWMServoDriver pwmRight(0x41);

Adafruit_PWMServoDriver* getDriver(uint8_t driver) {
    if (driver == ServoConfig::DRIVER_LEFT) {
        return &pwmLeft;
    }

    return &pwmRight;
}

ServoMotor createServo(LegID leg, ServoJoint joint) {
    const ServoConfigData& config =
        ServoConfig::SERVOS[leg][joint];

    return ServoMotor(
        getDriver(config.driver),
        config.channel,
        150,
        550,
        config.offset,
        config.inverted,
        config.minAngle,
        config.maxAngle
    );
}

ServoMotor servos[6][3] = {
    {
        createServo(LEG_LF, JOINT_COXA),
        createServo(LEG_LF, JOINT_FEMUR),
        createServo(LEG_LF, JOINT_TIBIA)
    },
    {
        createServo(LEG_LM, JOINT_COXA),
        createServo(LEG_LM, JOINT_FEMUR),
        createServo(LEG_LM, JOINT_TIBIA)
    },
    {
        createServo(LEG_LR, JOINT_COXA),
        createServo(LEG_LR, JOINT_FEMUR),
        createServo(LEG_LR, JOINT_TIBIA)
    },
    {
        createServo(LEG_RF, JOINT_COXA),
        createServo(LEG_RF, JOINT_FEMUR),
        createServo(LEG_RF, JOINT_TIBIA)
    },
    {
        createServo(LEG_RM, JOINT_COXA),
        createServo(LEG_RM, JOINT_FEMUR),
        createServo(LEG_RM, JOINT_TIBIA)
    },
    {
        createServo(LEG_RR, JOINT_COXA),
        createServo(LEG_RR, JOINT_FEMUR),
        createServo(LEG_RR, JOINT_TIBIA)
    }
};

const char* legNames[6] = {
    "LF",
    "LM",
    "LR",
    "RF",
    "RM",
    "RR"
};

void printPoint(const char* name, const Point3D& point) {
    Serial.print(name);
    Serial.print(": ");
    Serial.print(point.x, 3);
    Serial.print(", ");
    Serial.print(point.y, 3);
    Serial.print(", ");
    Serial.println(point.z, 3);
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Wire.begin();

    pwmLeft.begin();
    pwmRight.begin();

    pwmLeft.setPWMFreq(50);
    pwmRight.setPWMFreq(50);

    delay(500);

    const Point3D referenceLocal = {
        136.255f,
        0.0f,
        -108.306f
    };

    Serial.println();
    Serial.println("=== 6 LEG IK + SERVO TEST ===");

    for (int i = 0; i < 6; i++) {

        LegID leg = static_cast<LegID>(i);

        Point3D footB =
            localToBody(leg, referenceLocal);

        Point3D footLocal =
            bodyToLocal(leg, footB);

        IKResult result =
            calculateIK(footLocal);

        Serial.println();
        Serial.print("--- ");
        Serial.print(legNames[i]);
        Serial.println(" ---");

        printPoint("Body : ", footB);
        printPoint("Local: ", footLocal);

        if (!result.valid) {
            Serial.println("IK: INVALID");
            continue;
        }

        Serial.println("IK: VALID");

        Serial.print("Coxa:  ");
        Serial.println(result.angles.coxa, 4);

        Serial.print("Femur: ");
        Serial.println(result.angles.femur, 4);

        Serial.print("Tibia: ");
        Serial.println(result.angles.tibia, 4);

        servos[i][JOINT_COXA].setAngle(
            result.angles.coxa
        );

        servos[i][JOINT_FEMUR].setAngle(
            result.angles.femur
        );

        servos[i][JOINT_TIBIA].setAngle(
            result.angles.tibia
        );
    }

    Serial.println();
    Serial.println("=== TEST COMPLETE ===");
}

void loop() {
}