#ifndef GAIT_H
#define GAIT_H

#include "RobotGeometry.h"
#include "RobotController.h"
#include "Trajectory.h"

struct GaitParams {
    float stepPeriod;
    float stride;
    float stepHeight;
};

class Gait {
private:
    RobotController& robot;
    GaitParams params;
    Point3D homePositions[6];

    float phaseOffsets[6];

public:
    Gait(
        RobotController& controller,
        const GaitParams& gaitParams,
        const Point3D homes[6]
    );

    void update(float timeSeconds);
};

#endif