#include "Gait.h"

Gait::Gait(
    RobotController& controller,
    const GaitParams& gaitParams,
    const Point3D homes[6]
)
    : robot(controller),
      params(gaitParams)
{
    for (int i = 0; i < 6; i++) {
        homePositions[i] = homes[i];
    }

    phaseOffsets[LEG_LF] = 0.00f;
    phaseOffsets[LEG_LM] = 0.33f;
    phaseOffsets[LEG_LR] = 0.66f;

    phaseOffsets[LEG_RF] = 0.33f;
    phaseOffsets[LEG_RM] = 0.66f;
    phaseOffsets[LEG_RR] = 0.00f;
}

void Gait::update(float timeSeconds) {
    float globalPhase =
        fmod(
            timeSeconds,
            params.stepPeriod
        ) / params.stepPeriod;

    for (int i = 0; i < 6; i++) {
        LegID leg = static_cast<LegID>(i);

        float legPhase =
            fmod(
                globalPhase + phaseOffsets[i],
                1.0f
            );

        Point3D target =
            calculateTrajectory(
                homePositions[i],
                legPhase,
                {
                    params.stride,
                    params.stepHeight
                }
            );

        robot.setLegPosition(
            leg,
            target
        );
    }
}