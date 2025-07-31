#include "motor_steer_robot.h"

void MotorSteerRobot::init() {
    if (! CAN.available()){
        CAN.begin(CanBitRate::BR_500k);
    }
    
    while (!is_connected){
        if (CAN.available()) {
            is_connected = true;
            break;
        }
    }
}

void MotorSteerRobot::checkConnected() {
  if (! CAN.available()) {
    is_connected = false;
  }
  else {
    is_connected = true;
  }
}

void MotorSteerRobot::sendCommand(CommandSteeringRobot cmd) {
    switch (cmd.gear) {
    case GEAR_FORWARD:
        sendVelocity(cmd.velocity, id_drive_left);  // left drive motor
        sendVelocity(-cmd.velocity, id_drive_right); // right drive motor
        sendBrake(0);
        break;
    case GEAR_NEUTRAL:
        sendVelocity(0, id_drive_left);  // left drive motor
        sendVelocity(0, id_drive_right); // right drive motor
        sendBrake(0);
        break;
    case GEAR_REVERSE:
        sendVelocity(-cmd.velocity, id_drive_left);  // left drive motor
        sendVelocity(cmd.velocity, id_drive_right); // right drive motor
        sendBrake(0);
        break;
    case GEAR_NONE:
    default:
        sendVelocity(0, id_drive_left);  // left drive motor
        sendVelocity(0, id_drive_right); // right drive motor
        sendBrake(100);
        break;
    }
}

void MotorSteerRobot::sendVelocity(int vel, uint32_t can_id) {
    int rpm = vel * 0.01 * vel_to_rpm;
    
    uint8_t buffer[4] = {
    (uint8_t)((rpm >> 24) & 0xFF),
    (uint8_t)((rpm >> 16) & 0xFF),
    (uint8_t)((rpm >> 8) & 0xFF),
    (uint8_t)(rpm & 0xFF)
    };

    arduino::CanMsg msg(CanExtendedId(can_id), sizeof(buffer), buffer);
    CAN.write(msg);
}

void MotorSteerRobot::sendSteer(int steer, uint32_t can_id) {

}

void MotorSteerRobot::sendBrake(int brake) {

}