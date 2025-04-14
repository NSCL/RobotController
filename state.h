#ifndef STATE_H
#define STATE_H

enum State : uint8_t{
    INIT,
    MOTOR_IDLE,
    RF_IDLE,
    DRIVE_READY,
    DRIVE_IDLE,
    DRIVE_MANUAL,
    AUTO_IDLE,
    DRIVE_AUTO,
    AUTO_FAIL,
    ESTOP
};

#endif // STATE_H
