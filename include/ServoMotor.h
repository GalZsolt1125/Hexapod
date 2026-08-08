#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

class ServoMotor {
private:
    Adafruit_PWMServoDriver* driver;

    uint8_t channel;

    uint16_t pwmMin;
    uint16_t pwmMax;

    float offset;
    bool inverted;

    float minAngle;
    float maxAngle;

    float currentAngle;

    static constexpr float ServoCenter = 90.0f;

    float jointToServoAngle(float angle) const {
        float calibratedAngle = angle + offset;

        if (inverted) {
            calibratedAngle = -calibratedAngle;
        }

        return ServoCenter + calibratedAngle;
    }

    uint16_t angleToPWM(float servoAngle) const {
        servoAngle = constrain(servoAngle, 0.0f, 180.0f);

        float ratio = servoAngle / 180.0f;

        return (uint16_t)(
            pwmMin +
            ratio * (pwmMax - pwmMin)
        );
    }

public:
    ServoMotor(
        Adafruit_PWMServoDriver* pwmDriver = nullptr,
        uint8_t ch = 0,
        uint16_t minPulse = 150,
        uint16_t maxPulse = 550,
        float angleOffset = 0.0f,
        bool isInverted = false,
        float minDeg = -90.0f,
        float maxDeg = 90.0f
    )
        : driver(pwmDriver),
          channel(ch),
          pwmMin(minPulse),
          pwmMax(maxPulse),
          offset(angleOffset),
          inverted(isInverted),
          minAngle(minDeg),
          maxAngle(maxDeg),
          currentAngle(0.0f) {}

    bool isAngleValid(float angle) const {
        return angle >= minAngle && angle <= maxAngle;
    }

    void setAngle(float angle) {
        angle = constrain(angle, minAngle, maxAngle);

        currentAngle = angle;

        float servoAngle = jointToServoAngle(angle);
        uint16_t pulse = angleToPWM(servoAngle);

        if (driver != nullptr) {
            driver->setPWM(channel, 0, pulse);
        }
    }

    void setOffset(float newOffset) {
        offset = newOffset;
    }

    void setInverted(bool isInverted) {
        inverted = isInverted;
    }

    void setLimits(float minDeg, float maxDeg) {
        minAngle = minDeg;
        maxAngle = maxDeg;
    }

    void setPWMRange(uint16_t minPulse, uint16_t maxPulse) {
        pwmMin = minPulse;
        pwmMax = maxPulse;
    }

    float getAngle() const {
        return currentAngle;
    }

    float getOffset() const {
        return offset;
    }

    bool getInverted() const {
        return inverted;
    }

    float getMinAngle() const {
        return minAngle;
    }

    float getMaxAngle() const {
        return maxAngle;
    }
};

#endif