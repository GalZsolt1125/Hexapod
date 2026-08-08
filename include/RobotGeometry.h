#ifndef ROBOT_GEOMETRY_H
#define ROBOT_GEOMETRY_H

#include <cmath>

constexpr float MY_PI = 3.14159265359f;

struct Point3D {
    float x;
    float y;
    float z;
};

enum LegID {
    LEG_LF = 0,
    LEG_LM = 1,
    LEG_LR = 2,
    LEG_RF = 3,
    LEG_RM = 4,
    LEG_RR = 5
};

struct LegAttachment {
    Point3D offsetB;
    float mountAngleDeg;
};

namespace RobotGeometry {
    constexpr float Coxa_Length = 48.217f;
    constexpr float Femur_Length = 66.535f;
    constexpr float Tibia_Length = 113.806f;
}

constexpr float Ground_Z = -78.066f;

constexpr LegAttachment LEGS[6] = {
    { {-85.042f,  100.042f, 0.0f},  130.3666f },
    { {-102.950f,   0.000f, 0.0f},  180.0000f },
    { {-85.042f, -100.042f, 0.0f}, -130.3666f },

    { { 85.042f,  100.042f, 0.0f},   49.6334f },
    { {102.950f,   0.000f, 0.0f},    0.0000f },
    { { 85.042f, -100.042f, 0.0f},  -49.6334f }
};

inline Point3D bodyToLocal(LegID legId, const Point3D& bodyPoint) {
    const LegAttachment& att = LEGS[legId];

    float dx = bodyPoint.x - att.offsetB.x;
    float dy = bodyPoint.y - att.offsetB.y;
    float dz = bodyPoint.z - att.offsetB.z;

    float rad = -att.mountAngleDeg * MY_PI / 180.0f;
    float cosA = std::cos(rad);
    float sinA = std::sin(rad);

    Point3D localPoint;
    localPoint.x = dx * cosA - dy * sinA;
    localPoint.y = dx * sinA + dy * cosA;
    localPoint.z = dz;

    return localPoint;
}

inline Point3D localToBody(LegID legId, const Point3D& localPoint) {
    const LegAttachment& att = LEGS[legId];

    float rad = att.mountAngleDeg * MY_PI / 180.0f;
    float cosA = std::cos(rad);
    float sinA = std::sin(rad);

    float rx = localPoint.x * cosA - localPoint.y * sinA;
    float ry = localPoint.x * sinA + localPoint.y * cosA;

    Point3D bodyPoint;
    bodyPoint.x = rx + att.offsetB.x;
    bodyPoint.y = ry + att.offsetB.y;
    bodyPoint.z = localPoint.z + att.offsetB.z;

    return bodyPoint;
}

#endif