#include "rf_controller.h"

RFController::RFController(HardwareSerial& serial) : serialPort(serial) {}

void RFController::setChannel(uint8_t estopChannel, uint8_t disconnectChannel, uint8_t driveModeChannel) {
    CH_ESTOP = estopChannel;
    CH_DISCONNECT = disconnectChannel;
    CH_DRIVE_MODE = driveModeChannel;
}

bool RFController::checkChannel() {
    if (CH_ESTOP != CH_NULL && CH_DISCONNECT != CH_NULL) {
        return true;  // 값들이 유효하면 true 반환
    }
    return false;
}

bool RFController::begin() {
    if (!checkChannel()) {
        return false;
    }

    ibus.begin(serialPort);

    while (cnt_rec != 0) { // 첫 번째 iBus 메시지를 받을 때까지 대기
        ibus.loop();
        cnt_rec = ibus.cnt_rec;
        delay(100);
    }
    return true;
}

int RFController::readChannel(uint8_t channel, int minLimit, int maxLimit, int defaultValue) {
    uint16_t ch = ibus.readChannel(channel);
    if (ch >= 100) {
        return map(ch, 1000, 2000, minLimit, maxLimit);
    }
    return defaultValue;
}

int RFController::readThreeStageSwitch(uint8_t channel, int defaultValue) {
    uint16_t ch = ibus.readChannel(channel);
    if (ch >= 100) {
        if (ch > 1500) return -1; // 후진
        else if (ch < 1500) return 1; // 전진
        else return 0; // 중립
    }
    return defaultValue;
}

void RFController::checkConnection() {
    bool is_receiver_on = (cnt_rec != ibus.cnt_rec);
    if (is_receiver_on) {
        cnt_rec = ibus.cnt_rec;
    }
    bool is_transmitter_on = !readSwitch(CH_DISCONNECT, false);
    is_connected = is_receiver_on && is_transmitter_on;
}

bool RFController::readSwitch(uint8_t channel, bool defaultValue) {
    int intDefaultValue = (defaultValue) ? 100 : 0;
    int ch = readChannel(channel, 0, 100, intDefaultValue);
    return (ch > 50);
}

// =========================================================================
RFControllerDDRobot::RFControllerDDRobot(HardwareSerial& serial) : RFController(serial) {}

void RFControllerDDRobot::setChannel(uint8_t estopChannel, uint8_t disconnectChannel, uint8_t driveModeChannel, uint8_t velBrkChannel, uint8_t omegaChannel) {
    RFController::setChannel(estopChannel, disconnectChannel, driveModeChannel);
    CH_VEL_BRK = velBrkChannel;
    CH_OMEGA = omegaChannel;
}

void RFControllerDDRobot::setMaxOmega(float max_omega_) {
  max_omega = max_omega_ * 100;
}

void RFControllerDDRobot::setMaxVelocity(float max_vel_) {
  max_vel = max_vel_ * 100;
}

void RFControllerDDRobot::getCommand(CommandDDRobot &cmd) {
  // velocity
  cmd.velocity = readChannel(CH_VEL_BRK, -max_vel, max_vel, -5000) - 1;
  if (cmd.velocity <= 3 && cmd.velocity >= -3)
  {
    cmd.velocity = 0;
  }

  // omega
  cmd.omega = readChannel(CH_OMEGA, -max_omega, max_omega, -5000);
  if (cmd.omega <= 20 && cmd.omega >= -20)
  {
    cmd.omega = 0;
  }

  // estop
  cmd.estop = readSwitch(CH_ESTOP, false);

  // drive_mode
  bool isAuto = readSwitch(CH_DRIVE_MODE, false);
  cmd.drive_mode = isAuto ? MODE_AUTO : MODE_MANUAL;
}


// =========================================================================
RFControllerSteeringRobot::RFControllerSteeringRobot(HardwareSerial& serial) : RFController(serial) {}

void RFControllerSteeringRobot::setChannel(uint8_t estopChannel, uint8_t disconnectChannel, uint8_t gearChannel, uint8_t driveModeChannel, uint8_t velBrkChannel, uint8_t steerChannel) {
    RFController::setChannel(estopChannel, disconnectChannel, driveModeChannel);
    CH_VEL_BRK = velBrkChannel;
    CH_STEER = steerChannel;
    CH_GEAR = gearChannel;
}

void RFControllerSteeringRobot::setMaxSteer(float max_steer_) {
  max_steer = max_steer_ * 100;
}

void RFControllerSteeringRobot::setMaxVelocity(float max_vel_) {
  max_vel = max_vel_ * 100;
}

void RFControllerSteeringRobot::getCommand(CommandSteeringRobot &cmd) {
  // velocity
  int vel = readChannel(CH_VEL_BRK, -max_vel, max_vel, -5000) - 1;
  if (vel <= 3 && vel >= -3) // near zero
  {
    cmd.velocity = 0;
    cmd.brake = 0;
  }
  else if (vel < 0)
  {
    cmd.brake = -cmd.velocity;
    cmd.velocity = 0;
  }
  else
  {
    cmd.velocity = vel;
    cmd.brake = 0;
  }

  // omega
  cmd.steer = readChannel(CH_STEER, -max_steer, max_steer, -5000);
  if (cmd.steer <= 20 && cmd.steer >= -20)
  {
    cmd.steer = 0;
  }

  // gear
  int gear_ = readThreeStageSwitch(CH_GEAR, -100);
  switch (gear_) {
    case 1:
      cmd.gear = GEAR_FORWARD;
      break;
    case 0:
      cmd.gear = GEAR_NEUTRAL;
      break;
    case -1:
      cmd.gear = GEAR_REVERSE;
      break;
    case -100:
    default:
      cmd.gear = GEAR_NONE;
      break;
  }

  // estop
  cmd.estop = readSwitch(CH_ESTOP, false);

  // drive_mode
  bool isAuto = readSwitch(CH_DRIVE_MODE, false);
  cmd.drive_mode = isAuto ? MODE_AUTO : MODE_MANUAL;
}
