#ifndef RFCONTROLLER_H
#define RFCONTROLLER_H

#include <Arduino.h>
#include "IBusBM.h"
#include "util.h"

class RFController {
protected:
    IBusBM ibus;
    HardwareSerial& serialPort;

    uint8_t CH_NULL = 255;
    uint8_t CH_ESTOP = CH_NULL;
    uint8_t CH_DISCONNECT = CH_NULL;
    uint8_t CH_DRIVE_MODE;
    
    int cnt_rec = 0;
    
    virtual bool checkChannel();
    virtual bool readSwitch(uint8_t channel, bool defaultValue);
    virtual int readChannel(uint8_t channel, int minLimit, int maxLimit, int defaultValue);
    virtual int readThreeStageSwitch(uint8_t channel, int defaultValue);

public:
    // 생성자
    RFController(HardwareSerial& serial);

    bool is_connected = false;

    virtual void setChannel(uint8_t estopChannel, uint8_t disconnectChannel, uint8_t driveModeChannel);
    virtual bool begin();
    virtual void checkConnection();
};

class RFControllerDDRobot : public RFController {
private:
    uint8_t CH_VEL_BRK;
    uint8_t CH_OMEGA;
    uint16_t max_omega = 3000; // [deg] * 100
    uint16_t max_vel = 150; // [m/s] * 100
    
public:
    RFControllerDDRobot(HardwareSerial& serial);

    void setChannel(uint8_t estopChannel, uint8_t disconnectChannel, uint8_t driveModeChannel, uint8_t velBrkChannel, uint8_t omegaChannel);
    void setMaxOmega(float max_omega_);
    void setMaxVelocity(float max_vel_);

    // void getVelocity(int* velocity);
    // void getOmegaAngle(int* omega);

    void getCommand(CommandDDRobot& cmd);
};

class RFControllerSteeringRobot : public RFController {
private:
    uint8_t CH_VEL_BRK;
    uint8_t CH_STEER;
    uint8_t CH_GEAR;
    uint16_t max_steer = 3000; // [deg] * 100
    uint16_t max_vel = 150; // [m/s] * 100
    
public:
    RFControllerSteeringRobot(HardwareSerial& serial);

    void setChannel(uint8_t estopChannel, uint8_t disconnectChannel, uint8_t gearChannel, uint8_t driveModeChannel, uint8_t velBrkChannel, uint8_t steerChannel);
    void setMaxSteer(float max_steer_);
    void setMaxVelocity(float max_vel_);

    // void getVelocity(int* velocity);
    // void getOmegaAngle(int* omega);

    void getCommand(CommandSteeringRobot& cmd);
};

#endif // RFCONTROLLER_H