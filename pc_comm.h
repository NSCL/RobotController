#ifndef PC_COMM_H
#define PC_COMM_H

#include <Arduino.h>
#include "util.h"

const uint8_t BUFFER_SIZE = 32;

class PcComm {
  private:
    HardwareSerial *stream; 
  
    const byte STX[3] = {0x53, 0x54, 0x58};  // Start bytes
    const byte ETX[2] = {0x0D, 0x0A};        // End bytes

    byte buffer[BUFFER_SIZE];
    int bufferIndex = 0;
    unsigned long lastReceivedTime = 0;  // 마지막 데이터 수신 시간
    const uint8_t TIMEOUT_MS = 1000;

    byte alive_prev = 0;
    bool is_alive_on;
    bool is_upper_on;

    byte AorM = 0x00;
    byte ESTOP = 0x00;
    byte GEAR = 0x00;
    int SPEED = 0;
    int OMEGA = 0;
    byte BRAKE = 0x00;
    byte ALIVE = 0x00;

    void processPacket(byte *receivedPacket);

    void processIncomingData();
    bool isValidPacket(byte index);
    void checkConnectionTimeout();
    void updateConnectionStatus();

  public:
    // 생성자
    PcComm();

    bool is_connected;

    void begin(HardwareSerial& serial);
    void loop();

    void getCommandDD(CommandDDRobot &cmd);
    void getCommandSteer(CommandSteeringRobot &cmd);
    // void setMCUInfo(int omega_L, int omega_R, uint8_t battery_voltage);
    void sendPacket(byte AorM, byte ESTOP, byte GEAR, int SPEED, int OMEGA, byte BRAKE, int OMEGA_L, int OMEGA_R, byte BATTERY_V);

};

#endif


