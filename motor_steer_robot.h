#ifndef MOTOR_STEER_ROBOT_H
#define MOTOR_STEER_ROBOT_H

#include <Arduino_CAN.h>
#include <Arduino.h>
#include "util.h"

class MotorSteerRobot {
public:
    MotorSteerRobot(uint32_t id_drive_left_, uint32_t id_drive_right_, uint32_t id_steer_,
                    float wheel_radius_):
        id_drive_left(id_drive_left_),    // extended CAN ID
        id_drive_right(id_drive_right_),  // extended CAN ID
        id_steer(id_steer_),              // extended CAN ID
        wheel_radius(wheel_radius_) {}

    void init();
    void checkConnected();
    void sendCommand(CommandSteeringRobot cmd);

    bool is_connected = false;

private:
    uint32_t id_drive_left;
    uint32_t id_drive_right;
    uint32_t id_steer;
    float wheel_radius;
    float vel_to_rpm = (60.0 / (2.0 * 3.14159265358979323846 * wheel_radius));

    void sendVelocity(int vel, uint32_t can_id);
    void sendSteer(int steer, uint32_t can_id);
    void sendBrake(int brake);
};

#endif