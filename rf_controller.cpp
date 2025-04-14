#include "rf_controller.h"

RFController::RFController(HardwareSerial& serial) : serialPort(serial) {}

void RFController::setChannel(uint8_t estopChannel, uint8_t disconnectChannel, uint8_t gearChannel, uint8_t driveModeChannel) {
    CH_ESTOP = estopChannel;
    CH_DISCONNECT = disconnectChannel;
    CH_GEAR = gearChannel;
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

    while (cnt_rec == 0) { // 첫 번째 iBus 메시지를 받을 때까지 대기
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

bool RFController::isConnected() {
    bool is_receiver_on = (cnt_rec != ibus.cnt_rec);
    if (is_receiver_on) {
        cnt_rec = ibus.cnt_rec;
    }
    bool is_transmitter_on = !readSwitch(CH_DISCONNECT, false);
    return is_receiver_on && is_transmitter_on;
}

uint8_t RFController::getEstop() {
    return readSwitch(CH_ESTOP, false);
}

Gear RFController::getGear() {
    int _value = readThreeStageSwitch(CH_GEAR, -100);
    switch (_value) {
        case 1:
            return GEAR_FORWARD;
        case 0:
            return GEAR_NEUTRAL;
        case -1:
            return GEAR_REVERSE;
        case -100:
        default:
            return GEAR_NONE;
    }
}

DriveMode RFController::getDriveMode() {
    bool isAuto = readSwitch(CH_DRIVE_MODE, false);
    return isAuto ? MODE_AUTO : MODE_MANUAL;
}

bool RFController::readSwitch(uint8_t channel, bool defaultValue) {
    int intDefaultValue = (defaultValue) ? 100 : 0;
    int ch = readChannel(channel, 0, 100, intDefaultValue);
    return (ch > 50);
}