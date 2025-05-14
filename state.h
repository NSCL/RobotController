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

struct Command {
  int velocity = 0;
  int omega = 0;
};

#endif // STATE_H
