#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Arduino.h>
#include "RobotGeometry.h"

Point3D calculateTrajectory(
    const Point3D& home,
    float phase,
    float stride,
    float stepHeight,
    float motionX,
    float motionY,
    float rotation,
    LegID leg
);

#endif