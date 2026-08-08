#include "InverseKinematics.h"
#include <cmath>

namespace {

float clampFloat(float value, float minValue, float maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

}

IKResult calculateIK(const Point3D& footLocal) {
    IKResult result;

    const float coxa = RobotGeometry::Coxa_Length;
    const float femur = RobotGeometry::Femur_Length;
    const float tibia = RobotGeometry::Tibia_Length;

    float coxaAngle = std::atan2(
        footLocal.y,
        footLocal.x
    );

    float radialDistance = std::sqrt(
        footLocal.x * footLocal.x +
        footLocal.y * footLocal.y
    );

    float dx = radialDistance - coxa;

    float legDistance = std::sqrt(
        dx * dx +
        footLocal.z * footLocal.z
    );

    if (legDistance > femur + tibia) {
        return result;
    }

    if (legDistance < std::fabs(femur - tibia)) {
        return result;
    }

    float targetAngle = std::atan2(
        footLocal.z,
        dx
    );

    float femurCos = (
        femur * femur +
        legDistance * legDistance -
        tibia * tibia
    ) / (
        2.0f *
        femur *
        legDistance
    );

    femurCos = clampFloat(femurCos, -1.0f, 1.0f);

    float femurTriangleAngle = std::acos(femurCos);

    float kneeCos = (
        femur * femur +
        tibia * tibia -
        legDistance * legDistance
    ) / (
        2.0f *
        femur *
        tibia
    );

    kneeCos = clampFloat(kneeCos, -1.0f, 1.0f);

    float kneeInternalAngle = std::acos(kneeCos);

    float femurAngle =
        (targetAngle + femurTriangleAngle)
        * 180.0f / PI;

    float tibiaAngle =
        90.0f -
        kneeInternalAngle * 180.0f / PI;

    result.angles.coxa =
        coxaAngle * 180.0f / PI;

    result.angles.femur = femurAngle;
    result.angles.tibia = tibiaAngle;

    result.valid = true;

    return result;
}