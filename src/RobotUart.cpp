#include "RobotUart.h"

#include <cstring>

using namespace RobotProtocol;

RobotUart::RobotUart(
    HardwareSerial& serial,
    Robot& robot
)
    : _serial(serial),
      _robot(robot)
{
}

void RobotUart::begin()
{
    _state = ParserState::WAIT_HEADER;
    _lastValidPacketTime = millis();
    _failsafeActive = false;
    _txSequence = 0;
}

void RobotUart::update()
{
    while (_serial.available() > 0) {
        uint8_t byte = _serial.read();
        parseByte(byte);
    }

    if (
        !_failsafeActive &&
        (millis() - _lastValidPacketTime >
         FAILSAFE_TIMEOUT_MS)
    ) {
        _robot.stop();
        _failsafeActive = true;
    }
}

bool RobotUart::parseByte(uint8_t byte)
{
    switch (_state) {

        case ParserState::WAIT_HEADER:

            if (byte == FRAME_HEADER) {
                _state = ParserState::READ_TYPE;
            }

            break;


        case ParserState::READ_TYPE:

            if (!isValidCommandType(byte)) {
                resetParser();
                break;
            }

            _type = byte;
            _state = ParserState::READ_SEQ;

            break;


        case ParserState::READ_SEQ:

            _seq = byte;
            _state = ParserState::READ_LENGTH;

            break;


        case ParserState::READ_LENGTH:

            _length = byte;

            if (_length > MAX_PAYLOAD_SIZE) {
                resetParser();
                break;
            }

            _payloadIndex = 0;

            if (_length == 0) {
                _state = ParserState::READ_CRC;
            } else {
                _state = ParserState::READ_PAYLOAD;
            }

            break;


        case ParserState::READ_PAYLOAD:

            if (_payloadIndex >= MAX_PAYLOAD_SIZE) {
                resetParser();
                break;
            }

            _payload[_payloadIndex++] = byte;

            if (_payloadIndex >= _length) {
                _state = ParserState::READ_CRC;
            }

            break;


        case ParserState::READ_CRC:
        {
            _receivedCrc = byte;

            uint8_t crcData[
                3 + MAX_PAYLOAD_SIZE
            ];

            crcData[0] = _type;
            crcData[1] = _seq;
            crcData[2] = _length;

            for (uint8_t i = 0; i < _length; i++) {
                crcData[3 + i] = _payload[i];
            }

            uint8_t calculatedCrc =
                crc8(
                    crcData,
                    3 + _length
                );

            if (calculatedCrc == _receivedCrc) {

                _lastValidPacketTime = millis();
                _failsafeActive = false;

                executeCommand();

                resetParser();

                return true;
            }

            resetParser();

            break;
        }
    }

    return false;
}

void RobotUart::resetParser()
{
    _state = ParserState::WAIT_HEADER;

    _type = 0;
    _seq = 0;
    _length = 0;

    _payloadIndex = 0;
    _receivedCrc = 0;
}

bool RobotUart::readFloatPayload(float& value)
{
    if (_length != sizeof(FloatPayload)) {
        return false;
    }

    FloatPayload payload;

    memcpy(
        &payload,
        _payload,
        sizeof(FloatPayload)
    );

    value = payload.value;

    return true;
}

void RobotUart::executeCommand()
{
    CommandType command =
        static_cast<CommandType>(_type);

    switch (command) {

        case CommandType::START:

            if (_length != 0) {
                return;
            }

            _robot.start();

            break;


        case CommandType::STOP:

            if (_length != 0) {
                return;
            }

            _robot.stop();

            break;


        case CommandType::SET_MOTION:
        {
            if (_length != sizeof(MotionPayload)) {
                return;
            }

            MotionPayload payload;

            memcpy(
                &payload,
                _payload,
                sizeof(MotionPayload)
            );

            _robot.setMotion(
                payload.x,
                payload.y,
                payload.rot
            );

            break;
        }


        case CommandType::SET_BODY_X:
        {
            float value;

            if (!readFloatPayload(value)) {
                return;
            }

            _robot.setBodyX(value);

            break;
        }


        case CommandType::SET_BODY_Y:
        {
            float value;

            if (!readFloatPayload(value)) {
                return;
            }

            _robot.setBodyY(value);

            break;
        }


        case CommandType::SET_BODY_HEIGHT:
        {
            float value;

            if (!readFloatPayload(value)) {
                return;
            }

            _robot.setBodyHeight(value);

            break;
        }


        case CommandType::SET_BODY_OFFSET:
        {
            if (_length != sizeof(BodyOffsetPayload)) {
                return;
            }

            BodyOffsetPayload payload;

            memcpy(
                &payload,
                _payload,
                sizeof(BodyOffsetPayload)
            );

            _robot.setBodyOffset(
                payload.x,
                payload.y,
                payload.z
            );

            break;
        }


        case CommandType::SET_GAIT:
        {
            if (_length != sizeof(GaitPayload)) {
                return;
            }

            GaitPayload payload;

            memcpy(
                &payload,
                _payload,
                sizeof(GaitPayload)
            );

            switch (
                static_cast<GaitTypeWire>(
                    payload.type
                )
            ) {

                case GaitTypeWire::TRIPOD:

                    _robot.setGaitType(
                        GaitType::Tripod
                    );

                    break;


                case GaitTypeWire::WAVE:

                    _robot.setGaitType(
                        GaitType::Wave
                    );

                    break;


                case GaitTypeWire::DANCE:

                    _robot.setGaitType(
                        GaitType::Dance
                    );

                    break;


                default:

                    return;
            }

            break;
        }


        case CommandType::SET_STEP_LENGTH:
        {
            float value;

            if (!readFloatPayload(value)) {
                return;
            }

            _robot.setStepLength(value);

            break;
        }


        case CommandType::SET_STEP_HEIGHT:
        {
            float value;

            if (!readFloatPayload(value)) {
                return;
            }

            _robot.setStepHeight(value);

            break;
        }


        case CommandType::SET_STEP_PERIOD:
        {
            float value;

            if (!readFloatPayload(value)) {
                return;
            }

            if (value <= 0.05f) {
                return;
            }

            _robot.setStepPeriod(value);

            break;
        }


        case CommandType::GET_STATUS:

            if (_length != 0) {
                return;
            }

            sendStatus();

            break;


        case CommandType::STATUS:

            break;
    }
}

void RobotUart::sendPacket(
    CommandType type,
    const uint8_t* payload,
    uint8_t length
)
{
    if (length > MAX_PAYLOAD_SIZE) {
        return;
    }

    uint8_t sequence = _txSequence++;

    uint8_t crcData[
        3 + MAX_PAYLOAD_SIZE
    ];

    crcData[0] =
        static_cast<uint8_t>(type);

    crcData[1] = sequence;
    crcData[2] = length;

    for (uint8_t i = 0; i < length; i++) {
        crcData[3 + i] = payload[i];
    }

    uint8_t crc =
        crc8(
            crcData,
            3 + length
        );

    _serial.write(FRAME_HEADER);

    _serial.write(
        static_cast<uint8_t>(type)
    );

    _serial.write(sequence);

    _serial.write(length);

    if (length > 0) {
        _serial.write(
            payload,
            length
        );
    }

    _serial.write(crc);
}

void RobotUart::sendStatus()
{
    uint8_t payload[1];

    payload[0] =
        _robot.isMoving() ? 1 : 0;

    sendPacket(
        CommandType::STATUS,
        payload,
        sizeof(payload)
    );
}