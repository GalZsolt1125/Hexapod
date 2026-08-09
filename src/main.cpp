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
    const ServoConfigData& config = ServoConfig::SERVOS[leg][joint];

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

// Lépés paraméterei: Lépéshossz (stride), Lépésmagasság (stepHeight)
TrajectoryParams trajectory = {
    80.0f,
    80.0f
};

const float STEP_PERIOD = 1.4f; // Egy teljes lépésciklus ideje másodpercben

// Tripod fáziseltolások a 6 lábra
// 0.0f = 1. Tripod csoport (LF, LR, RM)
// 0.5f = 2. Tripod csoport (LM, RF, RR)
const float LEG_PHASE_OFFSETS[6] = {
    0.0f, // LEG_LF
    0.5f, // LEG_LM
    0.0f, // LEG_LR
    0.5f, // LEG_RF
    0.0f, // LEG_RM
    0.5f  // LEG_RR
};

Point3D homeBodyPositions[6];
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

    // Alapértelmezett kiindulási pozíciók kiszámítása mind a 6 lábra
    for (int i = 0; i < 6; i++) {
        Point3D homeLocal = {
            90.255f,
            0.0f,
            -100.306f
        };

        homeBodyPositions[i] = localToBody(
            static_cast<LegID>(i),
            homeLocal
        );
    }

    // Állítsuk az összes lábat az alaphelyzetbe
    robot.setAllLegsPosition(homeBodyPositions);

    startTime = millis();
}

void loop() {
    unsigned long elapsed = millis() - startTime;

    // Globális fázis [0.0 - 1.0)
    float globalPhase = fmod(
        elapsed / 1000.0f,
        STEP_PERIOD
    ) / STEP_PERIOD;

    // Mind a 6 láb pozíciójának frissítése
    for (int i = 0; i < 6; i++) {
        LegID leg = static_cast<LegID>(i);

        // Láb egyedi fázisa az eltolással
        float legPhase = fmod(
            globalPhase + LEG_PHASE_OFFSETS[i],
            1.0f
        );

        // Célpozíció kiszámítása az adott láb home pozíciójából kiindulva
        Point3D targetBody = calculateTrajectory(
            homeBodyPositions[i],
            legPhase,
            trajectory
        );

        // Mozgatás végrehajtása
        robot.setLegPosition(leg, targetBody);
    }

    // Smooth mozgás érdekében csökkentett késleltetés (50 Hz-es frissítés)
    delay(20);
}