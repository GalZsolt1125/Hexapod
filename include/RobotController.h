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

    Point3D legPositions[6];

    float bodyHeightOffset;


public:

    RobotController(
        ServoMotor (&coxa)[6],
        ServoMotor (&femur)[6],
        ServoMotor (&tibia)[6]
    ) {

        bodyHeightOffset = 0.0f;

        for (int i = 0; i < 6; i++) {

            servos[i][JOINT_COXA] = &coxa[i];
            servos[i][JOINT_FEMUR] = &femur[i];
            servos[i][JOINT_TIBIA] = &tibia[i];


            // Jelenlegi alapértelmezett lábpozíció.
            Point3D defaultLocal = {
                RobotGeometry::Coxa_Length + 88.038f,
                0.0f,
                -108.306f
            };


            legPositions[i] =
                localToBody(
                    static_cast<LegID>(i),
                    defaultLocal
                );
        }
    }

    bool setLegPosition(
        LegID leg,
        const Point3D& footB
    ) {

        Point3D footLocal =
            bodyToLocal(leg, footB);


        IKResult result =
            calculateIK(footLocal);


        if (!result.valid) {
            return false;
        }


        servos[leg][JOINT_COXA]->setAngle(
            result.angles.coxa
        );


        servos[leg][JOINT_FEMUR]->setAngle(
            result.angles.femur
        );


        servos[leg][JOINT_TIBIA]->setAngle(
            result.angles.tibia
        );


        legPositions[leg] = footB;


        return true;
    }

    bool moveLeg(
        LegID leg,
        const Point3D& delta
    ) {

        Point3D target =
            legPositions[leg];


        target.x += delta.x;
        target.y += delta.y;
        target.z += delta.z;


        return setLegPosition(
            leg,
            target
        );
    }

    bool setAllLegsPosition(
        const Point3D positions[6]
    ) {

        for (int i = 0; i < 6; i++) {

            if (!setLegPosition(
                static_cast<LegID>(i),
                positions[i]
            )) {

                return false;
            }
        }


        return true;
    }

    void setBodyHeight(
        float offset
    ) {

        bodyHeightOffset = offset;


        for (int i = 0; i < 6; i++) {

            Point3D target =
                legPositions[i];

            target.z =
                legPositions[i].z
                - bodyHeightOffset;


            setLegPosition(
                static_cast<LegID>(i),
                target
            );
        }
    }

    float getBodyHeight() const {

        return bodyHeightOffset;
    }

    Point3D getLegPosition(
        LegID leg
    ) const {

        return legPositions[leg];
    }
};

#endif