#include "pc_comm.h"

PcComm::PcComm() {}

void PcComm::begin(HardwareSerial& serial) {
  this->stream = &serial;
  stream->begin(115200);

  this->is_connected = true;
  this->is_alive_on = true;
}

void PcComm::sendPacket(byte AorM, byte ESTOP, byte GEAR, int SPEED, int OMEGA, byte BRAKE, int OMEGA_L, int OMEGA_R, byte BATTERY_V) {
  byte packet[19] = {
        STX[0], STX[1], STX[2],  // STX (3 bytes)
        AorM, ESTOP, GEAR,       // 단일 바이트 값들
        (byte)(SPEED >> 8), (byte)SPEED, // 2-byte int
        (byte)(OMEGA >> 8), (byte)OMEGA, // 2-byte int
        BRAKE,
        (byte)(OMEGA_L >> 8), (byte)OMEGA_L, // 2-byte int
        (byte)(OMEGA_R >> 8), (byte)OMEGA_R, // 2-byte int
        BATTERY_V, ALIVE,
        ETX[0], ETX[1]          // ETX (2 bytes)
    };
  stream->write(packet, 19);
}

void PcComm::processPacket(byte *receivedPacket) {
    PcComm::AorM = receivedPacket[3];
    PcComm::ESTOP = receivedPacket[4];
    PcComm::GEAR = receivedPacket[5];
    PcComm::SPEED = (receivedPacket[6] << 8) | receivedPacket[7];
    PcComm::OMEGA = (receivedPacket[8] << 8) | receivedPacket[9];
    PcComm::BRAKE = receivedPacket[10];
    PcComm::ALIVE = receivedPacket[11];

    this->is_alive_on = (alive_prev != ALIVE);
    this->alive_prev = ALIVE;

}

void PcComm::getCommand(int* speed, int* omega) {
  *speed = SPEED;
  *omega = OMEGA;
}

// void PcComm::setMCUInfo(int omega_L, int omega_R, uint8_t battery_voltage) {
//   this->omega_L = omega_L;
//   this->omega_R = omega_R;
//   this->battery_voltage = battery_voltage;
// }

void PcComm::loop() {
  processIncomingData();

  checkConnectionTimeout();

  updateConnectionStatus();

}

void PcComm::processIncomingData() {
  while (stream->available()) {
    byte incomingByte = stream->read();
    lastReceivedTime = millis();

    if (bufferIndex >= BUFFER_SIZE) {
      bufferIndex = 0;  // 버퍼 오버플로 방지
    }

    buffer[bufferIndex++] = incomingByte;

    // 최소한 패킷의 기본 길이(14바이트) 이상 수신됐을 때 처리
    if (bufferIndex >= 14) {
      for (int i = 0; i <= bufferIndex - 14; i++) {
        if (isValidPacket(i)) {
          processPacket(&buffer[i]);
          
          // 버퍼에서 해당 패킷을 제거
          memmove(buffer, buffer + i + 14, bufferIndex - (i + 14));
          bufferIndex -= (i + 14);
          break;
        }
      }
    }
  }
}

bool PcComm::isValidPacket(byte index) {
  return buffer[index] == STX[0] && buffer[index + 1] == STX[1] && buffer[index + 2] == STX[2] &&
         buffer[index + 12] == ETX[0] && buffer[index + 13] == ETX[1];
}

void PcComm::checkConnectionTimeout() {
  if (millis() - lastReceivedTime > TIMEOUT_MS) {
    is_upper_on = false;
  }
  else{
    is_upper_on = true;
  }
}

void PcComm::updateConnectionStatus() {
  if (!this->is_alive_on || !this->is_upper_on) {
    this->is_connected = false;
  } 
  else {
    this->is_connected = true;
  }
}

