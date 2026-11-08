
#include "IMU.h"

#include <Wire.h>
#include <math.h>

IMU::IMU()
    : ready(false),
      gyroBiasX(0.0f),
      gyroBiasY(0.0f),
      gyroBiasZ(0.0f),
      roll(0.0f),
      pitch(0.0f),
      lastUpdateUs(0)
{
    data = {};
}

bool IMU::writeRegister(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(MPU_ADDR);

    Wire.write(reg);
    Wire.write(value);

    return Wire.endTransmission() == 0;
}

bool IMU::readRegister(uint8_t reg, uint8_t& value)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)1) != 1) {
        return false;
    }

    value = Wire.read();

    return true;
}

bool IMU::readRaw(
    int16_t& ax,
    int16_t& ay,
    int16_t& az,
    int16_t& gx,
    int16_t& gy,
    int16_t& gz
)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14) != 14) {
        return false;
    }

    uint8_t buffer[14];

    for (uint8_t i = 0; i < 14; i++) {

        if (Wire.available() <= 0) {
            return false;
        }

        int value = Wire.read();

        if (value < 0) {
            return false;
        }

        buffer[i] = (uint8_t)value;
    }

    ax = (int16_t)(
        ((uint16_t)buffer[0] << 8) | buffer[1]
    );

    ay = (int16_t)(
        ((uint16_t)buffer[2] << 8) | buffer[3]
    );

    az = (int16_t)(
        ((uint16_t)buffer[4] << 8) | buffer[5]
    );

    gx = (int16_t)(
        ((uint16_t)buffer[8] << 8) | buffer[9]
    );

    gy = (int16_t)(
        ((uint16_t)buffer[10] << 8) | buffer[11]
    );

    gz = (int16_t)(
        ((uint16_t)buffer[12] << 8) | buffer[13]
    );

    return true;
}

bool IMU::begin()
{
    ready = false;

    uint8_t identity;

    if (!readRegister(0x75, identity)) {
        Serial.println("IMU: WHO_AM_I read error!");
        return false;
    }

    if (identity != 0x70) {
        Serial.printf(
            "IMU: Invalid WHO_AM_I: 0x%02X\n",
            identity
        );

        return false;
    }

    Serial.println("IMU: MPU-6500 detected");

    if (!writeRegister(0x6B, 0x80)) {
        return false;
    }

    delay(100);

    if (!writeRegister(0x6B, 0x01)) {
        return false;
    }

    if (!writeRegister(0x1B, 0x00)) {
        return false;
    }

    if (!writeRegister(0x1C, 0x00)) {
        return false;
    }

    if (!writeRegister(0x1A, GYRO_DLPF)) {
        return false;
    }

    if (!writeRegister(0x1D, ACCEL_DLPF)) {
        return false;
    }

    if (!writeRegister(0x19, SAMPLE_DIVIDER)) {
        return false;
    }

    delay(100);

    Serial.println("IMU: Configuration complete");

    if (!calibrate(500)) {
        Serial.println("IMU: Calibration failed!");
        return false;
    }

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    if (!readRaw(ax, ay, az, gx, gy, gz)) {
        Serial.println("IMU: Initial read failed!");
        return false;
    }

    float Ax = ax / ACCEL_SCALE;
    float Ay = ay / ACCEL_SCALE;
    float Az = az / ACCEL_SCALE;

    roll = atan2f(Ay, Az) * RAD_TO_DEG;

    pitch = atan2f(
        -Ax,
        sqrtf(Ay * Ay + Az * Az)
    ) * RAD_TO_DEG;

    data.roll = roll;
    data.pitch = pitch;

    lastUpdateUs = micros();

    ready = true;

    Serial.println("IMU: Initialization successful");

    return true;
}

bool IMU::calibrate(uint16_t samples)
{
    if (samples == 0) {
        return false;
    }

    int32_t sumGX = 0;
    int32_t sumGY = 0;
    int32_t sumGZ = 0;

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    uint16_t validSamples = 0;

    unsigned long startMs = millis();

    Serial.println();
    Serial.println("IMU: Gyroscope calibration");
    Serial.println("Keep the robot completely still!");

    while (validSamples < samples) {

        if (millis() - startMs > 10000) {
            Serial.println("IMU: Calibration timeout!");
            return false;
        }

        if (readRaw(ax, ay, az, gx, gy, gz)) {

            sumGX += gx;
            sumGY += gy;
            sumGZ += gz;

            validSamples++;
        }

        delay(5);
    }

    gyroBiasX =
        ((float)sumGX / samples) / GYRO_SCALE;

    gyroBiasY =
        ((float)sumGY / samples) / GYRO_SCALE;

    gyroBiasZ =
        ((float)sumGZ / samples) / GYRO_SCALE;


    Serial.println("IMU: Calibration complete");

    Serial.printf(
        "Gyro bias [deg/s]: X %.3f Y %.3f Z %.3f\n",
        gyroBiasX,
        gyroBiasY,
        gyroBiasZ
    );

    return true;
}

void IMU::update()
{
    if (!ready) {
        return;
    }

    int16_t rawAX, rawAY, rawAZ;
    int16_t rawGX, rawGY, rawGZ;

    if (!readRaw(
        rawAX, rawAY, rawAZ,
        rawGX, rawGY, rawGZ
    )) {
        return;
    }

    unsigned long nowUs = micros();

    float dt =
        (nowUs - lastUpdateUs) / 1000000.0f;

    lastUpdateUs = nowUs;

    if (dt <= 0.0f || dt > 0.1f) {
        return;
    }

    float Ax = rawAX / ACCEL_SCALE;
    float Ay = rawAY / ACCEL_SCALE;
    float Az = rawAZ / ACCEL_SCALE;

    float Gx = rawGX / GYRO_SCALE - gyroBiasX;
    float Gy = rawGY / GYRO_SCALE - gyroBiasY;
    float Gz = rawGZ / GYRO_SCALE - gyroBiasZ;

    float accelRoll =
        atan2f(Ay, Az) * RAD_TO_DEG;

    float accelPitch =
        atan2f(
            -Ax,
            sqrtf(Ay * Ay + Az * Az)
        ) * RAD_TO_DEG;

    roll =
        ALPHA * (roll + Gx * dt) +
        (1.0f - ALPHA) * accelRoll;

    pitch =
        ALPHA * (pitch + Gy * dt) +
        (1.0f - ALPHA) * accelPitch;

    data.accelX = Ax;
    data.accelY = Ay;
    data.accelZ = Az;

    data.gyroX = Gx;
    data.gyroY = Gy;
    data.gyroZ = Gz;

    data.gyroRawX = rawGX;
    data.gyroRawY = rawGY;
    data.gyroRawZ = rawGZ;

    data.roll = roll;
    data.pitch = pitch;

    data.dt = dt;
}

bool IMU::isReady() const
{
    return ready;
}

const IMUData& IMU::getData() const
{
    return data;
}
