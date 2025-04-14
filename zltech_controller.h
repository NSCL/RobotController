#ifndef ZLTECH_CONTROLLER_H
#define ZLTECH_CONTROLLER_H

#include <Arduino.h>
#include <Arduino_CAN.h>
#include <stdint.h>

class zltech_controller;

class PDOBase {
public:
    void init(zltech_controller* parent, uint8_t idx);
    void addMappedObject(uint16_t objectIdx, uint8_t subIdx, uint8_t bitLen);
    virtual void configurePDO(uint8_t transType, uint16_t eventTimer = 0) = 0;



protected:
    zltech_controller* canNode;
    uint8_t index;
    uint8_t nodeId;
    uint32_t mappingEntries[8];
    uint8_t  mappingCount = 0;

    uint16_t mappingBase(uint16_t base) const { return base + index; }
    uint16_t commBase(uint16_t base) const    { return base + index; }

    void writeObject(uint16_t index, uint8_t subIdx, uint32_t value, uint8_t bits);
};

class RPDO : public PDOBase {
private:
    uint32_t mappedValues[8] = {0};
public:
    uint32_t cobId;
    void init(zltech_controller* parent);
    void configurePDO(uint8_t transType, uint16_t eventTimer = 0) override;
    void setMappedValue(uint16_t index, uint8_t subIdx, uint32_t value);
    void send();

    void sendVel(int32_t leftVel, int32_t rightVel);
};

class TPDO : public PDOBase {
private:
    
    uint32_t mappedValues[8] = {0};
    uint32_t receivedValues[8] = {0};
public:
    uint32_t cobId;

    void init(zltech_controller* parent);
    void configurePDO(uint8_t transType, uint16_t eventTimer = 0) override;
    void onReceive(const uint8_t* data, uint8_t dlc);
    int32_t getMappedValue(uint16_t index, uint8_t subIdx) const;
};

enum Zltech_Motor_State : uint8_t{
    ZLTECH_MOTOR_INIT,
    ZLTECH_MOTOR_HEARTBEAT_SETUP,
    ZLTECH_MOTOR_VEL_MODE_SETUP,
    ZLTECH_MOTOR_AVAILABLE
};

class zltech_controller {
private:
    unsigned long _bitrate;
    unsigned long _lastHeartbeatTime = 0;
    unsigned long _lastSetupReadyTime = 0;
    bool _isMotorConnect = false;

    

public:
    uint8_t _nodeId;

    zltech_controller(uint8_t nodeId);
    
    CanMsg canread();
    
    bool init();
    void setNMT(uint8_t command);  // 0x01: operation, 0x02: stop, 0x80: pre-operation, 0x82: reset
    void writeObject(uint16_t index, uint8_t subIndex, uint32_t value, uint8_t sizeBytes);
    
    RPDO rpdo[4];
    TPDO tpdo[4];

    // 모터 velocity mode setup
    bool VelocityMode();
    void setVelocityMode(const CanMsg& msg);
    void sendVelocity(int32_t l_rpm, int32_t r_rpm);
    void readVelocity(const CanMsg& msg, int32_t* left_actual_rpm, int32_t* right_actual_rpm);

    // 하트비트 관련 
    // void sendHeartbeat(uint8_t state);
    // uint8_t readHeartbeat(const CanMsg& msg);
    void updateHeartbeatTimestamp();        // 하트비트 수신 시 갱신
    bool checkHeartbeatTimeout(uint32_t timeoutMs = 2000);  // 타임아웃 확인

    bool isConnected();
    void loop(const CanMsg& msg);

    Zltech_Motor_State _state = ZLTECH_MOTOR_INIT;
    // void setHeartbeat();
};

#endif // ZLTECH_CONTROLLER_H