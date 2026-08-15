#ifndef ROBOT_PROTOCOL_H
#define ROBOT_PROTOCOL_H

#include <Arduino.h>

namespace RobotProtocol {

constexpr uint8_t FRAME_HEADER = 0xAA;
constexpr uint8_t MAX_PAYLOAD_SIZE = 32;

enum class CommandType : uint8_t {

    START            = 0x01,
    STOP             = 0x02,
    SET_MOTION       = 0x03,

    SET_BODY_X       = 0x04,
    SET_BODY_Y       = 0x05,
    SET_BODY_HEIGHT  = 0x06,
    SET_BODY_OFFSET  = 0x07,

    SET_GAIT         = 0x08,
    SET_STEP_LENGTH  = 0x09,
    SET_STEP_HEIGHT  = 0x0A,
    SET_STEP_PERIOD  = 0x0B,

    CALIBRATION      = 0x0C,

    GET_STATUS       = 0x20,
    STATUS           = 0x21
};

enum class GaitTypeWire : uint8_t {
    TRIPOD = 0,
    WAVE   = 1,
    DANCE  = 2
};

struct MotionPayload {
    float x;
    float y;
    float rot;
};

struct FloatPayload {
    float value;
};

struct BodyOffsetPayload {
    float x;
    float y;
    float z;
};

struct GaitPayload {
    uint8_t type;
};

uint8_t crc8(
    const uint8_t* data,
    size_t length
);

bool isValidCommandType(
    uint8_t type
);

}

#endif