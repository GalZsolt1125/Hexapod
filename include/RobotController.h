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

    float bodyOffsetX;
    float bodyOffsetY;
    float bodyOffsetZ;


public:

    RobotController(
        ServoMotor (&coxa)[6],
        ServoMotor (&femur)[6],
        ServoMotor (&tibia)[6]
    ) {

        bodyOffsetX = 0.0f;
        bodyOffsetY = 0.0f;
        bodyOffsetZ = 0.0f;

        for (int i = 0; i < 6; i++) {

            servos[i][JOINT_COXA] = &coxa[i];
            servos[i][JOINT_FEMUR] = &femur[i];
            servos[i][JOINT_TIBIA] = &tibia[i];


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

    void setBodyX(
        float offset
    ) {

        float delta = offset - bodyOffsetX;
        bodyOffsetX = offset;

        for (int i = 0; i < 6; i++) {

            Point3D target =
                legPositions[i];

            target.x -= delta;

            setLegPosition(
                static_cast<LegID>(i),
                target
            );
        }
    }

    void setBodyY(
        float offset
    ) {

        float delta = offset - bodyOffsetY;
        bodyOffsetY = offset;

        for (int i = 0; i < 6; i++) {

            Point3D target =
                legPositions[i];

            target.y -= delta;

            setLegPosition(
                static_cast<LegID>(i),
                target
            );
        }
    }

    void setBodyHeight(
        float offset
    ) {

        float delta = offset - bodyOffsetZ;
        bodyOffsetZ = offset;

        for (int i = 0; i < 6; i++) {

            Point3D target =
                legPositions[i];

            target.z -= delta;

            setLegPosition(
                static_cast<LegID>(i),
                target
            );
        }
    }

    void setBodyOffset(
        float x,
        float y,
        float z
    ) {

        setBodyX(x);
        setBodyY(y);
        setBodyHeight(z);
    }

    float getBodyX() const {

        return bodyOffsetX;
    }

    float getBodyY() const {

        return bodyOffsetY;
    }

    float getBodyHeight() const {

        return bodyOffsetZ;
    }

    Point3D getLegPosition(
        LegID leg
    ) const {

        return legPositions[leg];
    }
};

#endif