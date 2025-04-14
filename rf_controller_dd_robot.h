#ifndef RFCONTROLLERDDROBOT_H
#define RFCONTROLLERDDROBOT_H

#include "rf_controller.h"

class RFControllerDDRobot : public RFController {
private:
    uint8_t CH_VEL_BRK;
    uint8_t CH_OMEGA;
    float max_vel = 2.0; // [m/s]
    float max_omega = 30.0; // [deg]

public:
    RFControllerDDRobot(HardwareSerial& serial);

    void setChannel(uint8_t estopChannel, uint8_t disconnectChannel, uint8_t gearChannel, uint8_t driveModeChannel, uint8_t velBrkChannel, uint8_t omegaChannel);
    void setMaxOmega(float max_omega_);
    void getVelocity(int* velocity);
    int getOmegaAngle();
};

#endif // RFCONTROLLERDDROBOT_H