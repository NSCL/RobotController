#ifndef RFCONTROLLER_H
#define RFCONTROLLER_H

#include <Arduino.h>
#include "IBusBM.h"
#include "gear.h"
#include "drive_mode.h"

class RFController {
protected:
    IBusBM ibus;
    HardwareSerial& serialPort;

    uint8_t CH_NULL = 255;
    uint8_t CH_ESTOP = CH_NULL;
    uint8_t CH_DISCONNECT = CH_NULL;
    uint8_t CH_GEAR;
    uint8_t CH_DRIVE_MODE;
    
    int cnt_rec = 0;
    bool is_channel_assigned = false;

public:
    // 생성자
    RFController(HardwareSerial& serial);

    virtual void setChannel(uint8_t estopChannel, uint8_t disconnectChannel, uint8_t gearChannel, uint8_t driveModeChannel);
    virtual bool checkChannel();
    virtual bool begin();
    virtual int readChannel(uint8_t channel, int minLimit, int maxLimit, int defaultValue);
    virtual int readThreeStageSwitch(uint8_t channel, int defaultValue);
    virtual bool isConnected();
    virtual Gear getGear();
    virtual DriveMode getDriveMode();
    virtual uint8_t getEstop();

protected:
    virtual bool readSwitch(uint8_t channel, bool defaultValue);
};

#endif // RFCONTROLLER_H