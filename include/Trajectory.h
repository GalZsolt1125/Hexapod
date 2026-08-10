#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Arduino.h>
#include "RobotGeometry.h"

struct TrajectoryParams {
    float stride;
    float stepHeight;
};

inline Point3D calculateTrajectory(
    const Point3D& home,
    float phase,
    const TrajectoryParams& params
) {
    phase = fmod(phase, 1.0f);

    if (phase < 0.0f) {
        phase += 1.0f;
    }

    Point3D target = home;

    if (phase < 0.5f) {
        float t = phase / 0.5f;

        target.y =
            home.y -
            params.stride * 0.5f +
            params.stride * t;

        target.z =
            home.z +
            params.stepHeight *
            sin(t * MY_PI);
    }
    else {
        float t = (phase - 0.5f) / 0.5f;

        target.y =
            home.y +
            params.stride * 0.5f -
            params.stride * t;

        target.z = home.z;
    }

    return target;
}

#endif