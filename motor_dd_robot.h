#ifndef MOTOR_DD_ROBOT_H
#define MOTOR_DD_ROBOT_H

#include <Arduino.h>
#include <Arduino_CAN.h>
#include <stdint.h>
#include "util.h"

class ZltechController;

class PDOBase {
public:
    void init(ZltechController* parent, uint8_t idx);
    void addMappedObject(uint16_t objectIdx, uint8_t subIdx, uint8_t bitLen);
    virtual void configurePDO(uint8_t transType, uint16_t eventTimer = 0) = 0;

protected:
    ZltechController* canNode;
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
    void init(ZltechController* parent);
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

    void init(ZltechController* parent);
    void configurePDO(uint8_t transType, uint16_t eventTimer = 0) override;
    void onReceive(const uint8_t* data, uint8_t dlc);
    int32_t getMappedValue(uint16_t index, uint8_t subIdx) const;
};

class ZltechController {
private:
    unsigned long _lastHeartbeatTime = 0;
    unsigned long _lastCANReceivedTime = 0;
    int32_t leftVel = 0; 
    int32_t rightVel = 0;

public:
    uint8_t node_id;
    float wheel_radius;
    float wheel_base;
    bool is_connected = false;

    ZltechController(uint8_t node_id_, float wheel_radius_, float wheel_base_):
                    node_id(node_id_),
                    wheel_radius(wheel_radius_),
                    wheel_base(wheel_base_) {}
        
    bool init();
    void setNMT(uint8_t command);  // 0x01: operation, 0x02: stop, 0x80: pre-operation, 0x82: reset
    void writeObject(uint16_t index, uint8_t subIndex, uint32_t value, uint8_t sizeBytes);
    
    RPDO rpdo[4];
    TPDO tpdo[4];

    // 모터 velocity mode setup
    bool VelocityMode();
    void sendVelocity(int32_t l_rpm, int32_t r_rpm);
    void readVelocity(int32_t* left_actual_rpm, int32_t* right_actual_rpm);
    void calcRPM(int target_vel, int target_omgega, int32_t* l_rpm, int32_t* r_rpm);
    void sendCommand(CommandDDRobot cmd);

    // 하트비트 관련 
    void updateHeartbeatTimestamp();        // 하트비트 수신 시 갱신
    bool checkHeartbeatTimeout(uint32_t timeoutMs = 2000);  // 타임아웃 확인

    // bool isConnected();
    void loop();
    void setup();
    // void can_init();
};

#endif // MOTOR_DD_ROBOT_H