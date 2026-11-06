#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

struct IMUData
{
    float accelX;
    float accelY;
    float accelZ;

    float gyroX;
    float gyroY;
    float gyroZ;

    float roll;
    float pitch;

    float dt;
};

class IMU
{
public:
    IMU();

    bool begin();
    void update();

    bool isReady() const;
    const IMUData& getData() const;

    void calibrate(uint16_t samples = 500);

private:
    IMUData data;

    bool ready;

    float gyroBiasX;
    float gyroBiasY;
    float gyroBiasZ;

    float roll;
    float pitch;

    unsigned long lastUpdateUs;

    bool readRaw(
        int16_t& ax,
        int16_t& ay,
        int16_t& az,
        int16_t& gx,
        int16_t& gy,
        int16_t& gz
    );

    bool writeRegister(uint8_t reg, uint8_t value);

    static constexpr uint8_t MPU_ADDR = 0x68;
    static constexpr float ALPHA = 0.98f;
};
#endif