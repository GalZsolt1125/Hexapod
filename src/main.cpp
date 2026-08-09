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

TrajectoryParams trajectory = {
    40.0f,
    30.0f
};

const float STEP_PERIOD = 2.0f;

Point3D homeBody =
    localToBody(
        LEG_LF,
        {
            136.255f,
            0.0f,
            -108.306f
        }
    );

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

    Point3D homePositions[6];

    for (int i = 0; i < 6; i++) {
        Point3D homeLocal = {
            136.255f,
            0.0f,
            -108.306f
        };

        homePositions[i] =
            localToBody(
                static_cast<LegID>(i),
                homeLocal
            );
    }

    robot.setAllLegsPosition(homePositions);

    startTime = millis();
}

void loop() {
    unsigned long elapsed =
        millis() - startTime;

    float phase =
        fmod(
            elapsed / 1000.0f,
            STEP_PERIOD
        ) / STEP_PERIOD;

    Point3D targetBody =
        calculateTrajectory(
            homeBody,
            phase,
            trajectory
        );

    Point3D targetLocal =
        bodyToLocal(
            LEG_LF,
            targetBody
        );

    Serial.print("Body X: ");
    Serial.print(targetBody.x, 3);

    Serial.print(" Y: ");
    Serial.print(targetBody.y, 3);

    Serial.print(" Z: ");
    Serial.println(targetBody.z, 3);

    Serial.print("Local X: ");
    Serial.print(targetLocal.x, 3);

    Serial.print(" Y: ");
    Serial.print(targetLocal.y, 3);

    Serial.print(" Z: ");
    Serial.println(targetLocal.z, 3);

    robot.setLegPosition(
        LEG_LF,
        targetBody
    );

    delay(100);
    }