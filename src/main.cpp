#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "RobotGeometry.h"
#include "RobotTypes.h"
#include "InverseKinematics.h"
#include "ServoMotor.h"
#include "ServoConfig.h"
#include "RobotController.h"
#include "Trajectory.h"
#include "Gait.h"

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

GaitParams gaitParams = {
    1.0f,
    60.0f,
    45.0f
};

Point3D homePositions[6];

Gait* gait;

unsigned long startTime;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Wire.begin();

    pwmLeft.begin();
    pwmRight.begin();

    pwmLeft.setPWMFreq(50);
    pwmRight.setPWMFreq(50);

    delay(500);

    for (int i = 0; i < 6; i++) {
        Point3D homeLocal = {
            100.255f,
            0.0f,
            -100.306f
        };

        homePositions[i] =
            localToBody(
                static_cast<LegID>(i),
                homeLocal
            );
    }

    robot.setAllLegsPosition(homePositions);

    gait = new Gait(
        robot,
        gaitParams,
        homePositions
    );

    startTime = millis();
}

void loop() {
    gait.setMotion(
        0.0f,
        1.0f,
        8.0f
    );

    float period = 0.8f;
    float phase = fmod((float)millis() / (period * 1000.0f), 1.0f);

    gait.update(robot, phase);

    delay(20);
}