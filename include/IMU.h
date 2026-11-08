
#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

struct IMUData
{
    // Accelerometer [g]
    float accelX;
    float accelY;
    float accelZ;

    // Gyroscope [deg/s] - bias corrected
    float gyroX;
    float gyroY;
    float gyroZ;

    // Raw gyroscope values [LSB]
    int16_t gyroRawX;
    int16_t gyroRawY;
    int16_t gyroRawZ;

    // Calculated angles [deg]
    float roll;
    float pitch;

    // Actual measurement interval [s]
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

    bool calibrate(uint16_t samples = 500);

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

    bool readRegister(uint8_t reg, uint8_t& value);

    static constexpr uint8_t MPU_ADDR = 0x68;

    static constexpr float ACCEL_SCALE = 16384.0f;
    static constexpr float GYRO_SCALE = 131.0f;

    static constexpr float ALPHA = 0.98f;

    static constexpr uint8_t GYRO_DLPF = 0x03;
    static constexpr uint8_t ACCEL_DLPF = 0x03;

    static constexpr uint8_t SAMPLE_DIVIDER = 4;
};

#endif