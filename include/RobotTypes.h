#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include "RobotGeometry.h"

struct JointAngles {
    float coxa = 0.0f;
    float femur = 0.0f;
    float tibia = 0.0f;
};

struct LegPose {
    Point3D footB;
};

struct RobotPose {
    Point3D bodyPosition;
    Point3D bodyRotation;
    LegPose legs[6];
};

#endif