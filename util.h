#ifndef UTIL_H
#define UTIL_H

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

enum Gear {
    GEAR_FORWARD,
    GEAR_NEUTRAL,
    GEAR_REVERSE,
    GEAR_NONE
};

enum DriveMode : uint8_t{
    MODE_MANUAL,
    MODE_AUTO,
    MODE_NONE
};

struct CommandDDRobot {
  int velocity = 0;
  int omega = 0;
  DriveMode drive_mode = MODE_MANUAL;
  bool estop = 0;
};

struct CommandSteeringRobot {
  uint16_t velocity = 0;
  int steer = 0;
  uint8_t brake = 0;
  Gear gear = GEAR_NEUTRAL;
  DriveMode drive_mode = MODE_MANUAL;
  bool estop = 0;
};

#endif // UTIL_H
