
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

    ax = (int16_t)((Wire.read() << 8) | Wire.read());
    ay = (int16_t)((Wire.read() << 8) | Wire.read());
    az = (int16_t)((Wire.read() << 8) | Wire.read());

    Wire.read();
    Wire.read();

    gx = (int16_t)((Wire.read() << 8) | Wire.read());
    gy = (int16_t)((Wire.read() << 8) | Wire.read());
    gz = (int16_t)((Wire.read() << 8) | Wire.read());

    return true;
}

bool IMU::begin()
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x75);

    if (Wire.endTransmission(false) != 0) {
        Serial.println("IMU: MPU-6500 nem talalhato!");
        return false;
    }

    if (Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)1) != 1) {
        Serial.println("IMU: WHO_AM_I olvasasi hiba!");
        return false;
    }

    uint8_t identity = Wire.read();

    if (identity != 0x70) {
        Serial.printf("IMU: hibas WHO_AM_I: 0x%02X\n", identity);
        return false;
    }

    if (!writeRegister(0x6B, 0x80)) return false;
    delay(100);

    if (!writeRegister(0x6B, 0x01)) return false;
    if (!writeRegister(0x1B, 0x00)) return false;
    if (!writeRegister(0x1C, 0x00)) return false;
    if (!writeRegister(0x1A, 0x03)) return false;
    if (!writeRegister(0x19, 0x04)) return false;

    delay(100);

    Serial.println("IMU: MPU-6500 OK");

    calibrate(500);

    int16_t ax, ay, az, gx, gy, gz;

    if (!readRaw(ax, ay, az, gx, gy, gz)) {
        Serial.println("IMU: adatolvasasi hiba!");
        return false;
    }

    float Ax = ax / 16384.0f;
    float Ay = ay / 16384.0f;
    float Az = az / 16384.0f;

    roll = atan2f(Ay, Az) * RAD_TO_DEG;

    pitch = atan2f(
        -Ax,
        sqrtf(Ay * Ay + Az * Az)
    ) * RAD_TO_DEG;

    data.roll = roll;
    data.pitch = pitch;

    lastUpdateUs = micros();
    ready = true;

    Serial.println("IMU: inicializalas kesz");

    return true;
}

void IMU::calibrate(uint16_t samples)
{
    int32_t sumGX = 0;
    int32_t sumGY = 0;
    int32_t sumGZ = 0;

    int16_t ax, ay, az, gx, gy, gz;

    Serial.println("IMU: gyro kalibralas...");
    Serial.println("A robot legyen mozdulatlan!");

    uint16_t validSamples = 0;

    while (validSamples < samples) {
        if (readRaw(ax, ay, az, gx, gy, gz)) {
            sumGX += gx;
            sumGY += gy;
            sumGZ += gz;
            validSamples++;
        }

        delay(2);
    }

    gyroBiasX = (float)sumGX / samples / 131.0f;
    gyroBiasY = (float)sumGY / samples / 131.0f;
    gyroBiasZ = (float)sumGZ / samples / 131.0f;

    Serial.println("IMU: kalibralas kesz");
}

void IMU::update()
{
    if (!ready) return;

    int16_t rawAX, rawAY, rawAZ;
    int16_t rawGX, rawGY, rawGZ;

    if (!readRaw(
        rawAX, rawAY, rawAZ,
        rawGX, rawGY, rawGZ
    )) {
        return;
    }

    unsigned long nowUs = micros();

    float dt = (nowUs - lastUpdateUs) / 1000000.0f;
    lastUpdateUs = nowUs;

    if (dt <= 0.0f || dt > 0.1f) return;

    float Ax = rawAX / 16384.0f;
    float Ay = rawAY / 16384.0f;
    float Az = rawAZ / 16384.0f;

    float Gx = rawGX / 131.0f - gyroBiasX;
    float Gy = rawGY / 131.0f - gyroBiasY;
    float Gz = rawGZ / 131.0f - gyroBiasZ;

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
