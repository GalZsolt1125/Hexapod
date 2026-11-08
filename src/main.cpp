#include <Arduino.h>
#include <Wire.h>

#include "Robot.h"
#include "RobotUart.h"
#include "IMU.h"

Robot robot;

HardwareSerial RobotSerial(1);

RobotUart robotUart(
    RobotSerial,
    robot
);

IMU imu;

constexpr uint32_t CONTROL_PERIOD_US = 20000; // 50 Hz

uint32_t lastControlUs = 0;

void setup()
{
    Serial.begin(115200);

    constexpr int UART_RX_PIN = 16;
    constexpr int UART_TX_PIN = 17;

    RobotSerial.begin(
        115200,
        SERIAL_8N1,
        UART_RX_PIN,
        UART_TX_PIN
    );

    Wire.begin(21, 22);
    Wire.setClock(400000);

    robot.begin();

    robot.setGaitType(
        GaitType::Tripod
    );

    robot.setStepLength(
        70.0f
    );

    robot.setStepHeight(
        35.0f
    );

    robot.setStepPeriod(
        0.8f
    );

    robot.setBodyOffset(
        0.0f,
        0.0f,
        -20.0f
    );

    Serial.println();
    Serial.println("Initializing IMU...");

    if (imu.begin()) {

        Serial.println("IMU READY");

    } else {

        Serial.println("WARNING: IMU unavailable!");
        Serial.println("Robot will run without IMU data.");
    }

    robotUart.begin();

    lastControlUs = micros();

    Serial.println();
    Serial.println("HEXAPOD INITIALIZATION COMPLETE");
    Serial.println("Control frequency: 50 Hz");
}

void loop()
{

    robotUart.update();

    uint32_t nowUs = micros();

    if ((uint32_t)(nowUs - lastControlUs) >= CONTROL_PERIOD_US)
    {
        lastControlUs += CONTROL_PERIOD_US;

        if ((uint32_t)(nowUs - lastControlUs) >= CONTROL_PERIOD_US) {
            lastControlUs = nowUs;
        }

        if (imu.isReady()) {
            imu.update();
        }

        robot.update();
    }
}