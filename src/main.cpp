#include <Arduino.h>

#include "Robot.h"
#include "RobotUart.h"

Robot robot;

HardwareSerial RobotSerial(1);

RobotUart robotUart(
    RobotSerial,
    robot
);

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

    robotUart.begin();
}

void loop()
{
    robotUart.update();

    robot.update();

    delay(20);
}