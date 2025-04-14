#include "zltech_controller.h"
#include "ZLAC8015D_OD.h"

#define WHEELSIZE  0.1651
#define WHEELBASE  0.4

zltech_controller::zltech_controller(uint8_t nodeId)
    : _nodeId(nodeId) {}

CanMsg zltech_controller::canread(){
    CanMsg msg = CAN.read();
    return msg;
}

bool zltech_controller::init() {
  if (!CAN.begin(CanBitRate::BR_500k)) {
        // Serial.println("CAN 초기화 실패");
        while(1);
        return false;
    }
    return true;

    // while (!CAN.available()) {
    //   CAN.begin(CanBitRate::BR_500k);
    //   delay(100);
    // }
    // return true;
}

void zltech_controller::setNMT(uint8_t command) {
    uint8_t nmt[2] = { command, _nodeId };
    CAN.write(CanMsg(CanStandardId(0x000), 2, nmt));
    delay(10);
}

void zltech_controller::updateHeartbeatTimestamp() {
    _lastHeartbeatTime = millis();
    _isMotorConnect = true;
}

bool zltech_controller::checkHeartbeatTimeout(uint32_t timeoutMs) {
    bool timedOut = (millis() - _lastHeartbeatTime > timeoutMs);
    return (timedOut)? true:false;

}

bool zltech_controller::isConnected(){
  return _isMotorConnect;
}

// void zltech_controller::setHeartbeat(){
  
//   _isHBReady = true;

// }

void zltech_controller::loop(const CanMsg& msg){
  if (msg.id == (0x700 + _nodeId) && msg.data_length == 1 && msg.data[0] == 0x00) {
    _lastSetupReadyTime = millis();
    _state = ZLTECH_MOTOR_VEL_MODE_SETUP;
  }

  switch (_state) {
    case ZLTECH_MOTOR_INIT: {
      _state = ZLTECH_MOTOR_HEARTBEAT_SETUP;
      break;
    }

    case ZLTECH_MOTOR_HEARTBEAT_SETUP: {
      // init();
      writeObject(0x1017, 0x00, 1000, 16); // 하트비트 100ms마다 전송
      // delay(100);
      _lastSetupReadyTime = millis();
      _state = ZLTECH_MOTOR_VEL_MODE_SETUP;
      break;
    } 

    case ZLTECH_MOTOR_VEL_MODE_SETUP: {
      if (millis() - _lastSetupReadyTime > 2000) {
        _state = ZLTECH_MOTOR_HEARTBEAT_SETUP;
        break;
      }

      if (msg.id == (0x700 + _nodeId) && msg.data_length == 1) {
        // if (msg.data[0] == 0x00 || msg.data[0] == 0x04) {
        if (msg.data[0] != 0x05) {
          VelocityMode();
        }
        _lastHeartbeatTime = millis();
        _isMotorConnect = true;
        _state = ZLTECH_MOTOR_AVAILABLE;
        break;
      }
      break;
    }

    case ZLTECH_MOTOR_AVAILABLE: {
      if (msg.id == (0x700 + _nodeId) && msg.data[0] == 0x05){
        updateHeartbeatTimestamp();
      }
      
      _isMotorConnect = ! checkHeartbeatTimeout(5000);

      if (!_isMotorConnect) {
        _state = ZLTECH_MOTOR_HEARTBEAT_SETUP;
        break;
      }

      break;
    }
  }
  
}

void zltech_controller::setVelocityMode(const CanMsg& msg){
    if (msg.id == (0x700 + _nodeId) && msg.data_length == 1 && msg.data[0] == 0x00) {
        VelocityMode();
        // Serial.println("[INFO] Node boot-up detected!");
    } 
}

bool zltech_controller::VelocityMode() {
    // Pre-Operational 상태로 전환
    setNMT(0x80);
    delay(50);

    // RPDO0 설정
    rpdo[0].init(this);
    rpdo[0].addMappedObject(Target_velocity, Left_Motor_Target_velocity, Target_velocity_Bits);
    rpdo[0].addMappedObject(Target_velocity, Right_Motor_Target_velocity, Target_velocity_Bits);
    rpdo[0].configurePDO(254); // 이벤트 기반

    // TPDO0 설정
    tpdo[0].init(this);
    tpdo[0].addMappedObject(Velocity_actual_value, Left_Motor_Velocity_actual_value, Velocity_actual_value_Bits);
    tpdo[0].addMappedObject(Velocity_actual_value, Right_Motor_Velocity_actual_value, Velocity_actual_value_Bits);
    tpdo[0].configurePDO(255, 100); // 주기 50ms

    // writeObject(0x1017, 0x00, 1000, 16);  // 16bit = 2 bytes
    // Serial.println("Heartbeat 전송 1000ms");

    delay(50);
    // Velocity 모드 설정
    writeObject(MODES_OF_OPERATION, 0x00, 3, 8); // 3 = Velocity mode

    writeObject(CONTROLWORD, 0x00, 0x06, CONTROLWORD_BITS); // Shutdown
    writeObject(CONTROLWORD, 0x00, 0x07, CONTROLWORD_BITS); // Switch On
    writeObject(CONTROLWORD, 0x00, 0x0F, CONTROLWORD_BITS); // Enable Operation

    delay(50);

    // Operational 상태로 전환
    setNMT(0x01);
    delay(100);

    // Serial.println("속도 모드 설정 완료");
    return true;
}

void zltech_controller::sendVelocity(int32_t l_rpm, int32_t r_rpm){
    rpdo[0].sendVel(l_rpm, r_rpm);
}

void zltech_controller::readVelocity(const CanMsg& msg, int32_t* left_actual_rpm, int32_t* right_actual_rpm){
    tpdo[0].onReceive(msg.data, msg.data_length);
    int32_t leftVel = tpdo[0].getMappedValue(Velocity_actual_value, Left_Motor_Velocity_actual_value); 
    int32_t rightVel = tpdo[0].getMappedValue(Velocity_actual_value, Right_Motor_Velocity_actual_value);

    *left_actual_rpm = leftVel * 0.1;
    *right_actual_rpm = rightVel * (-0.1);
}

void zltech_controller::writeObject(uint16_t index, uint8_t subIndex, uint32_t value, uint8_t sizeBytes) {
    uint8_t commandByte;
    uint8_t sdoFrame[8] = {0};

    switch (sizeBytes) {
        case 8:
            commandByte = 0x2F;
            memcpy(&sdoFrame[4], &value, 1); 
            break;
        case 16:
            commandByte = 0x2B;
            memcpy(&sdoFrame[4], &value, 2); 
            break;
        case 32:
            commandByte = 0x23;
            memcpy(&sdoFrame[4], &value, 4); 
            break;
        default:
            // Serial.println("Error: Invalid size for writeObject()");
            return;
    }

    sdoFrame[0] = commandByte;
    sdoFrame[1] = (uint8_t)(index & 0xFF);
    sdoFrame[2] = (uint8_t)((index >> 8) & 0xFF);
    sdoFrame[3] = subIndex;

    CAN.write(CanMsg(CanStandardId(0x600 + _nodeId), 8, sdoFrame));
    delay(10);
}


// PDOBASE Class
void PDOBase::init(zltech_controller* parent, uint8_t idx) {
    canNode = parent;
    index = idx;
    nodeId = parent->_nodeId;
    mappingCount = 0;
}

// PDOBase::writeObject
void PDOBase::writeObject(uint16_t index, uint8_t subIdx, uint32_t value, uint8_t bits) {
    canNode->writeObject(index, subIdx, value, bits);
}

void PDOBase::addMappedObject(uint16_t objectIdx, uint8_t subIdx, uint8_t bitLen) {
    if (mappingCount < 8) {
        uint32_t entry = ((uint32_t)objectIdx << 16) | ((uint16_t)subIdx << 8) | bitLen;
        mappingEntries[mappingCount++] = entry;
    } else {
        // Serial.println("Too many PDO mappings (max 8)");
    }
}


void RPDO::init(zltech_controller* parent) {
    uint8_t i = this - parent->rpdo;  // 현재 객체의 index 계산
    PDOBase::init(parent, i);
}

void RPDO::configurePDO(uint8_t transType, uint16_t eventTimer) {
    uint16_t mapIdx = mappingBase(0x1600);
    uint16_t comIdx = commBase(0x1400);
    this->cobId = 0x200 + (index * 0x100) + nodeId;
    

    writeObject(mapIdx, 0, 0, 8);

    for (size_t i = 0; i < mappingCount; ++i) {        
    writeObject(mapIdx, i + 1, mappingEntries[i], 32);
        }

    writeObject(mapIdx, 0, mappingCount, 8);
    writeObject(comIdx, 1, cobId, 32);
    writeObject(comIdx, 2, transType, 8);

    if (transType == 255 && eventTimer > 0) {
        writeObject(comIdx, 5, eventTimer, 16);
    }

    // Serial.println("[RPDO] 설정 완료");
}


void RPDO::setMappedValue(uint16_t index, uint8_t subIdx, uint32_t value) {
    for (uint8_t i = 0; i < mappingCount; ++i) {
        uint32_t entry = mappingEntries[i];
        uint16_t entIdx = (entry >> 16) & 0xFFFF;
        uint8_t entSubIdx = (entry >> 8) & 0xFF;

        if (entIdx == index && entSubIdx == subIdx) {
            mappedValues[i] = value;
            return;
        }
    }
    // Serial.println("unknown object");
}

void RPDO::send() {
    uint8_t pdoData[8] = {0};
    uint8_t bitOffset = 0;

    for (size_t i = 0; i < mappingCount; ++i) {
        uint32_t entry = mappingEntries[i];
        uint8_t bitLen = entry & 0xFF;
        uint8_t byteLen = bitLen / 8;

        if ((bitOffset / 8 + byteLen) <= 8) {
            memcpy(&pdoData[bitOffset / 8], &mappedValues[i], byteLen);
        } else {
            // Serial.println("");
            return;
        }

        bitOffset += bitLen;
    }

    uint8_t dlc = bitOffset / 8;
    CAN.write(CanMsg(CanStandardId(cobId), dlc, pdoData));
}

void RPDO::sendVel(int32_t leftVel, int32_t rightVel){
    uint8_t velData[8] = {0};
    memcpy(&velData[0],&leftVel,4);
    memcpy(&velData[4],&rightVel,4);

    // Serial.print("velData: ");
    // for (int i = 0; i < 8; ++i) {
    //     Serial.print(velData[i], HEX);
    //     Serial.print(" ");
    // }
    // Serial.println();

    CAN.write(CanMsg(CanStandardId(cobId), 8, velData));
}

// ===============================
// TPDO 클래스 구현
// ===============================

void TPDO::init(zltech_controller* parent) {
    uint8_t i = this - parent->tpdo;  // 현재 객체의 index 계산
    PDOBase::init(parent, i);
}

void TPDO::configurePDO(uint8_t transType, uint16_t eventTimer) {
    uint16_t mapIdx = mappingBase(0x1A00);
    uint16_t comIdx = commBase(0x1800);
    this->cobId = 0x180 + (index * 0x100) + nodeId;


    writeObject(mapIdx, 0, 0, 8);

    for (size_t i = 0; i < mappingCount; ++i) {        
        writeObject(mapIdx, i + 1, mappingEntries[i], 32);
        }

    writeObject(mapIdx, 0, mappingCount, 8);
    writeObject(comIdx, 1, cobId, 32);
    writeObject(comIdx, 2, transType, 8);

    if (transType == 255 && eventTimer > 0) {
        writeObject(comIdx, 5, eventTimer, 16);
    }

    // Serial.println("[TPDO] 설정 완료");
}


void TPDO::onReceive(const uint8_t* data, uint8_t dlc) {
    uint8_t bitOffset = 0;

    for (size_t i = 0; i < mappingCount; ++i) {
        uint32_t entry = mappingEntries[i];
        uint8_t bitLen = entry & 0xFF;
        uint8_t byteLen = bitLen / 8;

        if ((bitOffset / 8 + byteLen) > dlc) {
            // Serial.println("");
            return;
        }

        uint32_t value = 0;
        memcpy(&value, &data[bitOffset / 8], byteLen);
        receivedValues[i] = value;

        bitOffset += bitLen;
    }
}

int32_t TPDO::getMappedValue(uint16_t index, uint8_t subIdx) const {
    for (uint8_t i = 0; i < mappingCount; ++i) {
        uint32_t entry = mappingEntries[i];
        uint16_t entIdx = (entry >> 16) & 0xFFFF;
        uint8_t entSubIdx = (entry >> 8) & 0xFF;

        if (entIdx == index && entSubIdx == subIdx) {
            return (int32_t)receivedValues[i]; // 실제 값
        }
    }
    // Serial.println("");
    return 0;
}

