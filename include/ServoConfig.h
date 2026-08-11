#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include <Arduino.h>
#include "RobotGeometry.h"

enum ServoJoint {
    JOINT_COXA = 0,
    JOINT_FEMUR = 1,
    JOINT_TIBIA = 2
};

struct ServoConfigData {
    uint8_t driver;
    uint8_t channel;

    float offset;
    bool inverted;

    float minAngle;
    float maxAngle;
};

namespace ServoConfig {

constexpr uint8_t DRIVER_LEFT = 0;
constexpr uint8_t DRIVER_RIGHT = 1;

constexpr float DEFAULT_MIN = -90.0f;
constexpr float DEFAULT_MAX = 90.0f;

const ServoConfigData SERVOS[6][3] = {

    {
        { DRIVER_LEFT, 0, -5.0f, false,  DEFAULT_MIN, DEFAULT_MAX },
        { DRIVER_LEFT, 1, -4.0f, true,  DEFAULT_MIN, DEFAULT_MAX },
        { DRIVER_LEFT, 2,  0.0f, true,  DEFAULT_MIN, DEFAULT_MAX }
    },

    {
        { DRIVER_LEFT, 4, -7.0f, false,  DEFAULT_MIN, DEFAULT_MAX },
        { DRIVER_LEFT, 5,  2.0f, true,  DEFAULT_MIN, DEFAULT_MAX },
        { DRIVER_LEFT, 6, -5.0f, true,  DEFAULT_MIN, DEFAULT_MAX }
    },

    {
        { DRIVER_LEFT, 8, -6.0f, false,  DEFAULT_MIN, DEFAULT_MAX },
        { DRIVER_LEFT, 9, -5.0f, true,  DEFAULT_MIN, DEFAULT_MAX },
        { DRIVER_LEFT, 10, -6.0f, true,  DEFAULT_MIN, DEFAULT_MAX }
    },

    {
        { DRIVER_RIGHT, 15,  8.0f, false, DEFAULT_MIN, DEFAULT_MAX },
        { DRIVER_RIGHT, 14, -5.0f, false, DEFAULT_MIN, DEFAULT_MAX },
        { DRIVER_RIGHT, 13,  0.0f, false, DEFAULT_MIN, DEFAULT_MAX }
    },

    {
        { DRIVER_RIGHT, 11,  0.0f, false, DEFAULT_MIN, DEFAULT_MAX },
        { DRIVER_RIGHT, 10,  2.0f, false, DEFAULT_MIN, DEFAULT_MAX },
        { DRIVER_RIGHT, 9, -7.0f, false, DEFAULT_MIN, DEFAULT_MAX }
    },

    {
        { DRIVER_RIGHT, 7,  4.0f, false, DEFAULT_MIN, DEFAULT_MAX },
        { DRIVER_RIGHT, 6, -3.0f, false, DEFAULT_MIN, DEFAULT_MAX },
        { DRIVER_RIGHT, 5,  0.0f, false, DEFAULT_MIN, DEFAULT_MAX }
    }

};

}

#endif