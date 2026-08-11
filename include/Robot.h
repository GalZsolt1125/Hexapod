#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "RobotGeometry.h"
#include "RobotTypes.h"
#include "ServoMotor.h"
#include "ServoConfig.h"
#include "RobotController.h"
#include "Gait.h"


class Robot {

private:

    Adafruit_PWMServoDriver pwmLeft;
    Adafruit_PWMServoDriver pwmRight;

    ServoMotor coxa[6];
    ServoMotor femur[6];
    ServoMotor tibia[6];

    RobotController* controller;
    Gait gait;

    Point3D homeBodyPositions[6];
    float currentStepPeriod;

    bool moving;
    bool stopRequested;
    float currentPhase;
    unsigned long lastUpdateTime;

    float motionX;
    float motionY;
    float rotation;


public:

    Robot();

    void begin();

    void start();

    void stop();

    void calibration();

    bool isMoving() const;

    bool setLegPosition(
        LegID leg,
        const Point3D& position
    );

    bool setAllLegsPosition(
        const Point3D positions[6]
    );

    Point3D getLegPosition(
        LegID leg
    ) const;

    void setBodyX(float offset);

    void setBodyY(float offset);

    void setBodyHeight(float offset);

    void setBodyOffset(float x, float y, float z);

    float getBodyX() const;

    float getBodyY() const;

    float getBodyHeight() const;

    void setGaitType(GaitType type);

    void setStepLength(float length);

    void setStepHeight(float height);

    void setStepPeriod(float period);

    void setMotion(float x, float y, float rot);

    void update();
};

#endif