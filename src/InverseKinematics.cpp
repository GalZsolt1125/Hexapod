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

    float planarDistance = std::sqrt(
        dx * dx +
        footLocal.z * footLocal.z
    );

    if (planarDistance > femur + tibia) {
        return result;
    }

    if (planarDistance < std::fabs(femur - tibia)) {
        return result;
    }

    float targetAngle = std::atan2(
        footLocal.z,
        dx
    );

    float femurCos = (
        femur * femur +
        planarDistance * planarDistance -
        tibia * tibia
    ) / (
        2.0f *
        femur *
        planarDistance
    );

    femurCos = clampFloat(femurCos, -1.0f, 1.0f);

    float femurTriangleAngle = std::acos(femurCos);

    float kneeCos = (
        femur * femur +
        tibia * tibia -
        planarDistance * planarDistance
    ) / (
        2.0f *
        femur *
        tibia
    );

    kneeCos = clampFloat(kneeCos, -1.0f, 1.0f);

    float kneeInternalAngle = std::acos(kneeCos);

    result.angles.coxa =
        coxaAngle * 180.0f / PI;

    result.angles.femur =
        (targetAngle + femurTriangleAngle)
        * 180.0f / PI;

    result.angles.tibia =
        kneeInternalAngle * 180.0f / PI;

    result.valid = true;

    return result;
}