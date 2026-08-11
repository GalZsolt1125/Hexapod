#include "RobotProtocol.h"

namespace RobotProtocol {

// CRC8
// Polynomial: 0x07
// Initial value: 0x00

uint8_t crc8(const uint8_t* data, size_t length)
{
    uint8_t crc = 0x00;

    for (size_t i = 0; i < length; i++) {

        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8; bit++) {

            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

bool isValidCommandType(uint8_t type)
{
    switch (static_cast<CommandType>(type)) {

        case CommandType::START:
        case CommandType::STOP:
        case CommandType::SET_MOTION:
        case CommandType::SET_BODY_X:
        case CommandType::SET_BODY_Y:
        case CommandType::SET_BODY_HEIGHT:
        case CommandType::SET_BODY_OFFSET:
        case CommandType::SET_GAIT:
        case CommandType::SET_STEP_LENGTH:
        case CommandType::SET_STEP_HEIGHT:
        case CommandType::SET_STEP_PERIOD:
        case CommandType::CALIBRATION:
        case CommandType::GET_STATUS:
        case CommandType::STATUS:
            return true;

        default:
            return false;
    }
}

}