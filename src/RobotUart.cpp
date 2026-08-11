#include "RobotUart.h"

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
}

void RobotUart::update()
{
    while (_serial.available() > 0) {

        uint8_t byte = _serial.read();

        parseByte(byte);
    }

    if (
        !_failsafeActive &&
        (millis() - _lastValidPacketTime > FAILSAFE_TIMEOUT_MS)
    ) {

        _robot.stop();

        _failsafeActive = true;

        Serial.println(
            "[UART] FAILSAFE -> STOP"
        );
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

            _payload[_payloadIndex++] = byte;

            if (_payloadIndex >= _length) {

                _state = ParserState::READ_CRC;
            }

            break;

        case ParserState::READ_CRC:

            _receivedCrc = byte;

            {

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

                Serial.println(
                    "[UART] CRC ERROR"
                );

                resetParser();
            }

            break;
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

            Serial.println(
                "[UART] START"
            );

            break;

        case CommandType::STOP:

            if (_length != 0) {
                return;
            }

            _robot.stop();

            Serial.println(
                "[UART] STOP"
            );

            break;

        case CommandType::SET_MOTION:

            if (_length != sizeof(MotionPayload)) {
                return;
            }

            {

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

                Serial.printf(
                    "[UART] MOTION x=%.3f y=%.3f rot=%.3f\n",
                    payload.x,
                    payload.y,
                    payload.rot
                );
            }

            break;

        case CommandType::SET_BODY_X:

            if (_length != sizeof(FloatPayload)) {
                return;
            }

            {

                FloatPayload payload;

                memcpy(
                    &payload,
                    _payload,
                    sizeof(FloatPayload)
                );

                _robot.setBodyX(
                    payload.value
                );

                Serial.printf(
                    "[UART] BODY X = %.2f\n",
                    payload.value
                );
            }

            break;

        case CommandType::SET_BODY_Y:

            if (_length != sizeof(FloatPayload)) {
                return;
            }

            {

                FloatPayload payload;

                memcpy(
                    &payload,
                    _payload,
                    sizeof(FloatPayload)
                );

                _robot.setBodyY(
                    payload.value
                );

                Serial.printf(
                    "[UART] BODY Y = %.2f\n",
                    payload.value
                );
            }

            break;

        case CommandType::SET_BODY_HEIGHT:

            if (_length != sizeof(FloatPayload)) {
                return;
            }

            {

                FloatPayload payload;

                memcpy(
                    &payload,
                    _payload,
                    sizeof(FloatPayload)
                );

                _robot.setBodyHeight(
                    payload.value
                );

                Serial.printf(
                    "[UART] BODY HEIGHT = %.2f\n",
                    payload.value
                );
            }

            break;

        case CommandType::SET_BODY_OFFSET:

            if (_length != sizeof(BodyOffsetPayload)) {
                return;
            }

            {

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

                Serial.printf(
                    "[UART] BODY OFFSET %.2f %.2f %.2f\n",
                    payload.x,
                    payload.y,
                    payload.z
                );
            }

            break;

        case CommandType::SET_GAIT:

            if (_length != sizeof(GaitPayload)) {
                return;
            }

            {

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

                Serial.printf(
                    "[UART] GAIT = %u\n",
                    payload.type
                );
            }

            break;

        case CommandType::SET_STEP_LENGTH:

            if (_length != sizeof(FloatPayload)) {
                return;
            }

            {

                FloatPayload payload;

                memcpy(
                    &payload,
                    _payload,
                    sizeof(FloatPayload)
                );

                _robot.setStepLength(
                    payload.value
                );

                Serial.printf(
                    "[UART] STEP LENGTH = %.2f\n",
                    payload.value
                );
            }

            break;
        
        case CommandType::SET_STEP_HEIGHT:

            if (_length != sizeof(FloatPayload)) {
                return;
            }

            {

                FloatPayload payload;

                memcpy(
                    &payload,
                    _payload,
                    sizeof(FloatPayload)
                );

                _robot.setStepHeight(
                    payload.value
                );

                Serial.printf(
                    "[UART] STEP HEIGHT = %.2f\n",
                    payload.value
                );
            }

            break;

        case CommandType::SET_STEP_PERIOD:

            if (_length != sizeof(FloatPayload)) {
                return;
            }

            {

                FloatPayload payload;

                memcpy(
                    &payload,
                    _payload,
                    sizeof(FloatPayload)
                );

                _robot.setStepPeriod(
                    payload.value
                );

                Serial.printf(
                    "[UART] STEP PERIOD = %.3f\n",
                    payload.value
                );
            }

            break;

        case CommandType::CALIBRATION:

            if (_length != 0) {
                return;
            }

            _robot.calibration();

            Serial.println(
                "[UART] CALIBRATION"
            );

            break;

        case CommandType::GET_STATUS:

            if (_length != 0) {
                return;
            }

            Serial.println(
                "[UART] GET STATUS"
            );

            break;

        case CommandType::STATUS:

            break;
    }
}