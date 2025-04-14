#include "rf_controller_dd_robot.h"

RFControllerDDRobot::RFControllerDDRobot(HardwareSerial& serial) : RFController(serial) {}

void RFControllerDDRobot::setChannel(uint8_t estopChannel, uint8_t disconnectChannel, uint8_t gearChannel, uint8_t driveModeChannel, uint8_t velBrkChannel, uint8_t omegaChannel) {
    RFController::setChannel(estopChannel, disconnectChannel, gearChannel, driveModeChannel);
    CH_VEL_BRK = velBrkChannel;
    CH_OMEGA = omegaChannel;
}

void RFControllerDDRobot::setMaxOmega(float max_omega_) {
  max_omega = max_omega_;
}

void RFControllerDDRobot::getVelocity(int* velocity) {
  int val = readChannel(CH_VEL_BRK, -200, 200, -1000) - 1;
  // *velocity = val > 0 ? val : 0;
  if (val <= 3 && val >= -3)
  {
    val = 0;
  }
  *velocity = val;
}

int RFControllerDDRobot::getOmegaAngle() {
  int omega = readChannel(CH_OMEGA, -2000, 2000, -1000);
  if (omega <= 20 && omega >= -20)
  {
    omega = 0;
  }
  
  return omega;
}