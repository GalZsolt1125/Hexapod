#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include "RobotTypes.h"

struct IKResult {
    bool valid = false;
    JointAngles angles;
};

IKResult calculateIK(const Point3D& footLocal);

#endif