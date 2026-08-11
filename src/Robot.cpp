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
        getDriver(
            left,
            right,
            config.driver
        ),

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
          createServo(
              pwmLeft,
              pwmRight,
              LEG_LF,
              JOINT_COXA
          ),

          createServo(
              pwmLeft,
              pwmRight,
              LEG_LM,
              JOINT_COXA
          ),

          createServo(
              pwmLeft,
              pwmRight,
              LEG_LR,
              JOINT_COXA
          ),

          createServo(
              pwmLeft,
              pwmRight,
              LEG_RF,
              JOINT_COXA
          ),

          createServo(
              pwmLeft,
              pwmRight,
              LEG_RM,
              JOINT_COXA
          ),

          createServo(
              pwmLeft,
              pwmRight,
              LEG_RR,
              JOINT_COXA
          )
      },


      femur{
          createServo(
              pwmLeft,
              pwmRight,
              LEG_LF,
              JOINT_FEMUR
          ),

          createServo(
              pwmLeft,
              pwmRight,
              LEG_LM,
              JOINT_FEMUR
          ),

          createServo(
              pwmLeft,
              pwmRight,
              LEG_LR,
              JOINT_FEMUR
          ),

          createServo(
              pwmLeft,
              pwmRight,
              LEG_RF,
              JOINT_FEMUR
          ),

          createServo(
              pwmLeft,
              pwmRight,
              LEG_RM,
              JOINT_FEMUR
          ),

          createServo(
              pwmLeft,
              pwmRight,
              LEG_RR,
              JOINT_FEMUR
          )
      },


      tibia{
          createServo(
              pwmLeft,
              pwmRight,
              LEG_LF,
              JOINT_TIBIA
          ),

          createServo(
              pwmLeft,
              pwmRight,
              LEG_LM,
              JOINT_TIBIA
          ),

          createServo(
              pwmLeft,
              pwmRight,
              LEG_LR,
              JOINT_TIBIA
          ),

          createServo(
              pwmLeft,
              pwmRight,
              LEG_RF,
              JOINT_TIBIA
          ),

          createServo(
              pwmLeft,
              pwmRight,
              LEG_RM,
              JOINT_TIBIA
          ),

          createServo(
              pwmLeft,
              pwmRight,
              LEG_RR,
              JOINT_TIBIA
          )
      },


      controller(nullptr)
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
}


bool Robot::setLegPosition(
    LegID leg,
    const Point3D& position
) {

    return controller->setLegPosition(
        leg,
        position
    );
}


bool Robot::setAllLegsPosition(
    const Point3D positions[6]
) {

    return controller->setAllLegsPosition(
        positions
    );
}


Point3D Robot::getLegPosition(
    LegID leg
) const {

    return controller->getLegPosition(
        leg
    );
}


void Robot::setBodyX(
    float offset
) {

    controller->setBodyX(
        offset
    );
}


void Robot::setBodyY(
    float offset
) {

    controller->setBodyY(
        offset
    );
}


void Robot::setBodyHeight(
    float offset
) {

    controller->setBodyHeight(
        offset
    );
}


void Robot::setBodyOffset(
    float x,
    float y,
    float z
) {

    controller->setBodyOffset(
        x,
        y,
        z
    );
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