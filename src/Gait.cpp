#include "Gait.h"
#include <Arduino.h>
#include <math.h>

Gait::Gait() {

    // Tripod alapértelmezett
    phaseOffsets[LEG_LF] = 0.00f;
    phaseOffsets[LEG_LM] = 0.50f;
    phaseOffsets[LEG_LR] = 0.00f;

    phaseOffsets[LEG_RF] = 0.50f;
    phaseOffsets[LEG_RM] = 0.00f;
    phaseOffsets[LEG_RR] = 0.50f;
}

void Gait::setHomePositions(
    const Point3D positions[6]
) {
    for (int i = 0; i < 6; i++) {
        LegID leg = static_cast<LegID>(i);

void Gait::setType(GaitType type) {

    gaitType = type;

    if (type == GaitType::Tripod) {

        phaseOffsets[LEG_LF] = 0.00f;
        phaseOffsets[LEG_LM] = 0.50f;
        phaseOffsets[LEG_LR] = 0.00f;

        phaseOffsets[LEG_RF] = 0.50f;
        phaseOffsets[LEG_RM] = 0.00f;
        phaseOffsets[LEG_RR] = 0.50f;
    }

    else if (type == GaitType::Wave) {

        phaseOffsets[LEG_LF] = 0.00f;
        phaseOffsets[LEG_LM] = 0.33f;
        phaseOffsets[LEG_LR] = 0.66f;

        phaseOffsets[LEG_RF] = 0.33f;
        phaseOffsets[LEG_RM] = 0.66f;
        phaseOffsets[LEG_RR] = 0.00f;
    }

    else if (type == GaitType::Dance) {

        phaseOffsets[LEG_LF] = 0.00f;
        phaseOffsets[LEG_LM] = 0.00f;
        phaseOffsets[LEG_LR] = 0.00f;

        phaseOffsets[LEG_RF] = 0.50f;
        phaseOffsets[LEG_RM] = 0.50f;
        phaseOffsets[LEG_RR] = 0.50f;
    }
}

void Gait::setStepLength(float length) {
    stepLength = length;
}

void Gait::setStepHeight(float height) {
    stepHeight = height;
}

void Gait::setStepPeriod(float period) {
    stepPeriod = period;
}

void Gait::setMotion(
    float x,
    float y,
    float rot
) {
    motionX = x;
    motionY = y;
    rotation = rot;
}

void Gait::update(
    Robot& robot,
    float phase
) {
    for (int i = 0; i < 6; i++) {

        LegID leg =
            static_cast<LegID>(i);

        float legPhase =
            fmod(
                phase + phaseOffsets[i],
                1.0f
            );

        Point3D target =
            calculateTrajectory(
                homePositions[i],
                legPhase,
                stepLength,
                stepHeight,
                motionX,
                motionY,
                rotation,
                leg
            );

        robot.setLegPosition(
            leg,
            target
        );
    }
}