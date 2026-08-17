#include "Robot.h"


namespace {

Adafruit_PWMServoDriver* getDriver(
    Adafruit_PWMServoDriver& left,
    Adafruit_PWMServoDriver& right,
    uint8_t driver
) {

    if (driver == ServoConfig::DRIVER_LEFT) {
        return &left;
    }

    return &right;
}

ServoMotor createServo(
    Adafruit_PWMServoDriver& left,
    Adafruit_PWMServoDriver& right,
    LegID leg,
    ServoJoint joint
) {

    const ServoConfigData& config =
        ServoConfig::SERVOS[leg][joint];

    return ServoMotor(
        getDriver(left, right, config.driver),
        config.channel,
        150,
        550,
        config.offset,
        config.inverted,
        config.minAngle,
        config.maxAngle
    );
}

}


Robot::Robot()

    : pwmLeft(0x40),
      pwmRight(0x41),

      coxa{
          createServo(pwmLeft, pwmRight, LEG_LF, JOINT_COXA),
          createServo(pwmLeft, pwmRight, LEG_LM, JOINT_COXA),
          createServo(pwmLeft, pwmRight, LEG_LR, JOINT_COXA),
          createServo(pwmLeft, pwmRight, LEG_RF, JOINT_COXA),
          createServo(pwmLeft, pwmRight, LEG_RM, JOINT_COXA),
          createServo(pwmLeft, pwmRight, LEG_RR, JOINT_COXA)
      },

      femur{
          createServo(pwmLeft, pwmRight, LEG_LF, JOINT_FEMUR),
          createServo(pwmLeft, pwmRight, LEG_LM, JOINT_FEMUR),
          createServo(pwmLeft, pwmRight, LEG_LR, JOINT_FEMUR),
          createServo(pwmLeft, pwmRight, LEG_RF, JOINT_FEMUR),
          createServo(pwmLeft, pwmRight, LEG_RM, JOINT_FEMUR),
          createServo(pwmLeft, pwmRight, LEG_RR, JOINT_FEMUR)
      },

      tibia{
          createServo(pwmLeft, pwmRight, LEG_LF, JOINT_TIBIA),
          createServo(pwmLeft, pwmRight, LEG_LM, JOINT_TIBIA),
          createServo(pwmLeft, pwmRight, LEG_LR, JOINT_TIBIA),
          createServo(pwmLeft, pwmRight, LEG_RF, JOINT_TIBIA),
          createServo(pwmLeft, pwmRight, LEG_RM, JOINT_TIBIA),
          createServo(pwmLeft, pwmRight, LEG_RR, JOINT_TIBIA)
      },

      controller(nullptr),
      currentStepPeriod(0.8f),
      moving(false),
      stopRequested(false),
      currentPhase(0.0f),
      lastUpdateTime(0),
      motionX(0.0f),
      motionY(0.0f),
      rotation(0.0f)
{

    controller =
        new RobotController(
            coxa,
            femur,
            tibia
        );
}


void Robot::begin() {

    Wire.begin();

    pwmLeft.begin();
    pwmRight.begin();

    pwmLeft.setPWMFreq(50);
    pwmRight.setPWMFreq(50);

    delay(500);

    for (int i = 0; i < 6; i++) {

        Point3D homeLocal = {
            95.255f,
            0.0f,
            -146.306f
        };

        homeBodyPositions[i] =
            localToBody(
                static_cast<LegID>(i),
                homeLocal
            );
    }

    setAllLegsPosition(homeBodyPositions);

    gait.setHomePositions(homeBodyPositions);
    gait.setType(GaitType::Tripod);
    gait.setStepLength(80.0f);
    gait.setStepHeight(40.0f);
    gait.setStepPeriod(currentStepPeriod);

    lastUpdateTime = millis();
}


void Robot::start() {
    moving = true;
    stopRequested = false;
}


void Robot::stop() {
    if (moving) {
        stopRequested = true;
    }
}


bool Robot::isMoving() const {
    return moving;
}


bool Robot::setLegPosition(LegID leg, const Point3D& position) {
    return controller->setLegPosition(leg, position);
}


bool Robot::setAllLegsPosition(const Point3D positions[6]) {
    return controller->setAllLegsPosition(positions);
}


Point3D Robot::getLegPosition(LegID leg) const {
    return controller->getLegPosition(leg);
}


void Robot::setBodyX(float offset) {
    controller->setBodyX(offset);
}


void Robot::setBodyY(float offset) {
    controller->setBodyY(offset);
}


void Robot::setBodyHeight(float offset) {
    controller->setBodyHeight(offset);
}


void Robot::setBodyOffset(float x, float y, float z) {
    controller->setBodyOffset(x, y, z);
}


float Robot::getBodyX() const {
    return controller->getBodyX();
}


float Robot::getBodyY() const {
    return controller->getBodyY();
}


float Robot::getBodyHeight() const {
    return controller->getBodyHeight();
}


void Robot::setGaitType(GaitType type) {
    gait.setType(type);
}


void Robot::setStepLength(float length) {
    gait.setStepLength(length);
}


void Robot::setStepHeight(float height) {
    gait.setStepHeight(height);
}


void Robot::setStepPeriod(float period) {
    currentStepPeriod = period;
    gait.setStepPeriod(period);
}


void Robot::setMotion(float x, float y, float rot) {
    motionX = x;
    motionY = y;
    rotation = rot;

    // Ha van érdemi mozgási parancs, elindítjuk a lépést.
    // Ha nincs (joystick alaphelyzetben), kérjük a forgás/léptetés leállítását.
    if (fabs(x) > 0.01f || fabs(y) > 0.01f || fabs(rot) > 0.01f) {
        start();
    } else {
        stop();
    }
}


void Robot::update() {

    unsigned long now = millis();
    float dt = (now - lastUpdateTime) / 1000.0f;
    lastUpdateTime = now;

    if (dt <= 0.0f || dt > 0.5f) {
        dt = 0.02f;
    }

    if (moving) {

        currentPhase += dt / currentStepPeriod;

        if (currentPhase >= 1.0f) {
            currentPhase = fmod(currentPhase, 1.0f);

            if (stopRequested) {
                moving = false;
                stopRequested = false;
                currentPhase = 0.0f;
            }
        }

        if (stopRequested) {
            gait.setMotion(0.0f, 0.0f, 0.0f);
        } else {
            gait.setMotion(motionX, motionY, rotation);
        }

        gait.update(*this, currentPhase);

    } else {

        gait.setMotion(0.0f, 0.0f, 0.0f);
        gait.update(*this, 0.0f);
    }
}