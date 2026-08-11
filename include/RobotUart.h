#ifndef ROBOT_UART_H
#define ROBOT_UART_H

#include <Arduino.h>
#include "RobotProtocol.h"
#include "Robot.h"

class RobotUart {

public:

    RobotUart(
        HardwareSerial& serial,
        Robot& robot
    );

    void begin();
    void update();

private:

    enum class ParserState : uint8_t {

        WAIT_HEADER,

        READ_TYPE,
        READ_SEQ,
        READ_LENGTH,

        READ_PAYLOAD,

        READ_CRC
    };

    bool parseByte(uint8_t byte);

    void resetParser();

    void executeCommand();

    bool readFloatPayload(float& value);

    HardwareSerial& _serial;
    Robot& _robot;

    ParserState _state = ParserState::WAIT_HEADER;

    uint8_t _type = 0;
    uint8_t _seq = 0;
    uint8_t _length = 0;

    uint8_t _payload[RobotProtocol::MAX_PAYLOAD_SIZE];
    uint8_t _payloadIndex = 0;

    uint8_t _receivedCrc = 0;

    uint32_t _lastValidPacketTime = 0;

    bool _failsafeActive = false;

    static constexpr uint32_t FAILSAFE_TIMEOUT_MS = 250;
};

#endif