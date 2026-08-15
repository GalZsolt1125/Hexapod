#ifndef GAIT_H
#define GAIT_H

#include "RobotGeometry.h"
#include "Trajectory.h"

class Robot;

enum class GaitType {
    Tripod,
    Wave,
    Dance
};

class Gait {

private:

    Point3D homePositions[6];

    GaitType gaitType = GaitType::Tripod;

    float stepLength = 80.0f;
    float stepHeight = 40.0f;
    float stepPeriod = 1.0f;

    float motionX = 0.0f;
    float motionY = 0.0f;
    float rotation = 0.0f;

    float phaseOffsets[6];

public:

    Gait();

    void setHomePositions(
        const Point3D positions[6]
    );

    void setType(
        GaitType type
    );

    void setStepLength(
        float length
    );

    void setStepHeight(
        float height
    );

    void setStepPeriod(
        float period
    );

    void setMotion(
        float x,
        float y,
        float rotation
    );

    void update(
        Robot& robot,
        float phase
    );
};

#endif