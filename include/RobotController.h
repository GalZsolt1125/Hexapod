#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "RobotGeometry.h"
#include "RobotTypes.h"
#include "InverseKinematics.h"
#include "ServoMotor.h"
#include "ServoConfig.h"

class RobotController {
private:
    ServoMotor* servos[6][3];

public:
    RobotController(
        ServoMotor coxa[6],
        ServoMotor femur[6],
        ServoMotor tibia[6]
    ) {
        for (int i = 0; i < 6; i++) {
            servos[i][JOINT_COXA]  = &coxa[i];
            servos[i][JOINT_FEMUR] = &femur[i];
            servos[i][JOINT_TIBIA] = &tibia[i];
        }
    }

    bool setLegPosition(LegID leg, const Point3D& footB) {
        Point3D footLocal = bodyToLocal(leg, footB);
        IKResult result = calculateIK(footLocal);

        if (!result.valid) return false;

        servos[leg][JOINT_COXA]->setAngle(result.angles.coxa);
        servos[leg][JOINT_FEMUR]->setAngle(result.angles.femur);
        servos[leg][JOINT_TIBIA]->setAngle(result.angles.tibia);

        return true;
    }

    bool setAllLegsPosition(const Point3D footPositions[6]) {
        bool valid = true;

        for (int i = 0; i < 6; i++) {
            if (!setLegPosition(static_cast<LegID>(i), footPositions[i])) {
                valid = false;
            }
        }

        return valid;
    }
};

#endif
