#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "RobotGeometry.h"
#include "RobotTypes.h"
#include "InverseKinematics.h"
#include "ServoMotor.h"
#include "ServoConfig.h"
#include "RobotController.h"

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

ServoMotor coxa[6] = {
    createServo(LEG_LF, JOINT_COXA),
    createServo(LEG_LM, JOINT_COXA),
    createServo(LEG_LR, JOINT_COXA),
    createServo(LEG_RF, JOINT_COXA),
    createServo(LEG_RM, JOINT_COXA),
    createServo(LEG_RR, JOINT_COXA)
};

ServoMotor femur[6] = {
    createServo(LEG_LF, JOINT_FEMUR),
    createServo(LEG_LM, JOINT_FEMUR),
    createServo(LEG_LR, JOINT_FEMUR),
    createServo(LEG_RF, JOINT_FEMUR),
    createServo(LEG_RM, JOINT_FEMUR),
    createServo(LEG_RR, JOINT_FEMUR)
};

ServoMotor tibia[6] = {
    createServo(LEG_LF, JOINT_TIBIA),
    createServo(LEG_LM, JOINT_TIBIA),
    createServo(LEG_LR, JOINT_TIBIA),
    createServo(LEG_RF, JOINT_TIBIA),
    createServo(LEG_RM, JOINT_TIBIA),
    createServo(LEG_RR, JOINT_TIBIA)
};

RobotController robot(coxa, femur, tibia);

void setup() {
    Serial.begin(115200);
    delay(1000);

    Wire.begin();

    pwmLeft.begin();
    pwmRight.begin();

    pwmLeft.setPWMFreq(50);
    pwmRight.setPWMFreq(50);

    delay(500);

    const Point3D reference = {
        136.255f,
        50.0f,
        -108.306f
    };

    Point3D targets[6];

    for (int i = 0; i < 6; i++) {
        targets[i] =
            localToBody(
                static_cast<LegID>(i),
                reference
            );
    }

    bool valid =
        robot.setAllLegsPosition(targets);

    Serial.println();

    if (valid) {
        Serial.println("ROBOT POSITION: VALID");
    } else {
        Serial.println("ROBOT POSITION: INVALID");
    }
}

void loop() {
}