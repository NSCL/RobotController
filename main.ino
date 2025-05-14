#include "rf_controller_dd_robot.h"
#include "pc_comm.h"
#include "state.h"
#include "gear.h"
#include "drive_mode.h"

#include <Arduino_CAN.h>
#include "zltech_controller.h"
#include "ZLAC8015D_OD.h"

#include "mbed.h"

// === Motor Driver ===
#define NODE_ID 1
zltech_controller zltech(NODE_ID);
CanMsg zltech_msg;

// === Robot Hardware ===
#define WHEEL_R  0.08255
#define WHEELBASE  0.4

// === Portenta Hardware ===
#define LED_PIN LED_BUILTIN
#define SERIAL_RF Serial3
#define SERIAL_PC Serial
#define SERIAL_DEBUG Serial1
#define PIN_ESTOP 6
#define voltagePin A0

// === RF Controller Channel ===
#define CH_ESTOP 7
#define CH_DISCONNECT 5
#define CH_VELOCITY 1
#define CH_OMEGA 3
#define CH_GEAR 6
#define CH_DRIVE_MODE 4

// === Variables ===
State state = INIT;
Gear gear = GEAR_NEUTRAL;
DriveMode drive_mode = MODE_MANUAL;
uint8_t estop = 0;

Command cmd_rf;
Command cmd_auto;

uint8_t heartbeatState = 255;
int32_t left_actual_rpm = 0;
int32_t right_actual_rpm = 0;
byte brake = 0;
uint8_t battery = 140;

RFControllerDDRobot rf_controller(SERIAL_RF);
PcComm upper_comm;

bool is_estop = false;
bool is_estop_btn_on = false;
bool is_estop_rf_on = false;
bool is_rf_disconnected = false;
bool is_upper_connected = false;
bool is_motor_connected = false;
bool is_battery_low = false;

// === Threads & Mutex ===
rtos::Thread threadRF; // RF Controller Thread
rtos::Thread threadRobotControl; // Motor Control Thread
rtos::Thread threadUpperComm; // PC <-> Portenta Communication Thread
rtos::Thread threadBattery; // Battery check
rtos::Mutex dataMutex;

#define FLAG_RF (1UL << 0)
#define FLAG_ROBOT_CONTROL (1UL << 1)
#define FLAG_UPPER_COMM (1UL << 2)
#define FLAG_BATTERY (1UL << 3)

mbed::Ticker tick1ms;   /* 10 ms*/
uint32_t tick_cnt = 0;
const uint8_t tick_robot_control = 10; // 10 ms
const uint8_t tick_rf = 20; // 20 ms
const uint8_t tick_upper = 2; // 2 ms
const uint8_t tick_battery = 1000; // 1000 ms
rtos::EventFlags ef;

void setup() {
  
  SERIAL_DEBUG.begin(9600);
  delay(50);

  upper_comm.begin(SERIAL_PC);
  delay(50);

  state = MOTOR_IDLE;
  
  zltech.init();
  zltech.setup();
  is_motor_connected = zltech.isConnected();
  delay(50);
  
  state = RF_IDLE;
  rf_controller.setMaxOmega(20.0); // [deg]
  rf_controller.setMaxVelocity(1.5); // [m/s]
  rf_controller.setChannel(CH_ESTOP, CH_DISCONNECT, CH_GEAR, CH_DRIVE_MODE, CH_VELOCITY, CH_OMEGA); // estop, disconnect, gear, drive_mode, throttle, steer,  channels
  rf_controller.begin();

  state = DRIVE_READY;

  pinMode(PIN_ESTOP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ESTOP), detectEstopSW, CHANGE);

  // Thread
  tick1ms.attach(&setFlag, 0.001); // 1 ms

  threadRF.start(taskRF);
  threadRobotControl.start(taskRobotControl);
  threadUpperComm.start(taskUpperComm);
  threadBattery.start(taskBattery);
}

void taskRF(){
  while(true){
    ef.wait_any(FLAG_RF);

    is_rf_disconnected = !rf_controller.isConnected();

    if (! is_rf_disconnected) {
      is_estop_rf_on = rf_controller.getEstop();
      drive_mode = rf_controller.getDriveMode();

      rf_controller.getVelocity(&cmd_rf.velocity);
      cmd_rf.omega = rf_controller.getOmegaAngle() ;
      gear = rf_controller.getGear();
    }

    // SERIAL_DEBUG.print("[");
    // SERIAL_DEBUG.print(!is_rf_disconnected ? "RECEIVED" : "DISCONNECTED");
    // SERIAL_DEBUG.print("] ");
    // SERIAL_DEBUG.print("Estop: ");
    // SERIAL_DEBUG.print(is_estop_rf_on);
    // SERIAL_DEBUG.print(" | Drive Mode: ");
    // SERIAL_DEBUG.print(drive_mode);
    // SERIAL_DEBUG.print(" | state: ");
    // SERIAL_DEBUG.print(state);
    // SERIAL_DEBUG.print(" | Velocity: ");
    // SERIAL_DEBUG.print(velocity);
    // SERIAL_DEBUG.print(" | Omega : ");
    // SERIAL_DEBUG.print(omega);
    // SERIAL_DEBUG.print(" | Gear: ");
    // SERIAL_DEBUG.print(gear);
    // SERIAL_DEBUG.print(" | State: ");
    // SERIAL_DEBUG.print(stateToString(state));
    // SERIAL_DEBUG.print(" | Battery: ");
    // SERIAL_DEBUG.println(battery);
    
  }
}

// Motor control 
void taskRobotControl(){
  while(true){
    ef.wait_any(FLAG_ROBOT_CONTROL);
    
    // DD Robot
    int32_t l_rpm = 0;
    int32_t r_rpm = 0;
    zltech.loop();
    is_motor_connected = zltech.isConnected();

    // Common
    is_estop = (is_estop_btn_on || is_estop_rf_on || is_rf_disconnected || is_battery_low);

    if (! is_motor_connected) state = MOTOR_IDLE;
    else if (is_estop) state = ESTOP;

    switch (state) {
      case ESTOP: {
        if (!is_estop) {
          state = DRIVE_READY;
          break;
        }

        break;
      }
      
      case MOTOR_IDLE: {
        if (is_motor_connected) {
          state = DRIVE_READY;
        }

        // DD Robot
        zltech.setup();
        break;
      }

      case RF_IDLE: {
        if (!is_rf_disconnected){
          state = DRIVE_READY;
        }
        break;
      }

      case DRIVE_READY: {
        state = (drive_mode == MODE_MANUAL) ? DRIVE_MANUAL : AUTO_IDLE;
        break;
      }

      case DRIVE_MANUAL: {
        if (drive_mode != MODE_MANUAL) {
          state = DRIVE_READY;
          break;
          }
        // velocity, omega
        // Motor Control  
        calcRPM(cmd_rf.velocity, cmd_rf.omega,
                &l_rpm, &r_rpm);

        break;
      }

      case AUTO_IDLE: {
        if (drive_mode != MODE_AUTO) state = DRIVE_READY;

        state = (is_upper_connected) ? DRIVE_AUTO : AUTO_FAIL;

        break;
      }

      case DRIVE_AUTO: {
        if (drive_mode != MODE_AUTO) state = DRIVE_READY;

        calcRPM(cmd_auto.velocity, cmd_auto.omega,
                &l_rpm, &r_rpm);
                
        break;
      }
      
      case AUTO_FAIL: {
        if (drive_mode != MODE_AUTO) state = DRIVE_READY;

        if (is_upper_connected) state = DRIVE_AUTO;
        break;
      }

      default: {
        break;
      }
    }

    zltech.sendVelocity(l_rpm, -r_rpm);
    zltech.readVelocity(&left_actual_rpm, &right_actual_rpm);
  }
}

void taskUpperComm(){
  while(true){
    ef.wait_any(FLAG_UPPER_COMM);

    upper_comm.loop();
    upper_comm.sendPacket(drive_mode, estop, gear, cmd_rf.velocity, cmd_rf.omega, brake, left_actual_rpm, right_actual_rpm, battery);

    is_upper_connected = upper_comm.is_connected;

    if (is_upper_connected) {
      upper_comm.getCommand(&cmd_auto.velocity, &cmd_auto.omega);   
    }

    else {
      cmd_auto.velocity = 0;
      cmd_auto.omega = 0;
    }
    // if (is_upper_connected) {
    //   digitalWrite(LED_PIN, HIGH);  // 정상 데이터 수신 중이면 LED OFF
    // } 
    // else {
    //   if (millis() % 500 < 250) {
    //     digitalWrite(LED_PIN, HIGH);  // 0.25초 켜짐
    //   } else {
    //     digitalWrite(LED_PIN, LOW);   // 0.25초 꺼짐
    //   }
    // }
  }
}

void taskBattery(){
  while(true){
    ef.wait_any(FLAG_BATTERY);

    uint16_t sensorValue = analogRead(voltagePin);
    battery = (sensorValue * 310UL) / 1023 - 100;

    is_battery_low = (battery < 120);

  }
}

void setFlag(){
  if(tick_cnt % tick_rf == 0) ef.set(FLAG_RF);
  if(tick_cnt % tick_robot_control == 0) ef.set(FLAG_ROBOT_CONTROL);
  if(tick_cnt % tick_upper == 0) ef.set(FLAG_UPPER_COMM);
  if(tick_cnt % tick_battery == 0) ef.set(FLAG_BATTERY);
  ++tick_cnt;
}

void detectEstopSW() {
    is_estop_btn_on = (digitalRead(PIN_ESTOP) == LOW);
}

void calcRPM(int target_vel, int target_omgega, 
              int32_t* l_rpm, int32_t* r_rpm) {
  float v; // m/s * 100 scale
  float w; // rad/s * 1000 scale
  v = (float)(target_vel) / 100.0f; // -3 ~ 3[m/s]
  w = (float)(target_omgega) / 1000.0f; // -2 ~ 2 [rad/s]
  float delta_vel = w * (WHEELBASE / 2.0); // [m/s]

  float v_left = v - delta_vel; // [m/s]
  float v_right = v + delta_vel; // [m/s]

  float rpm_left = v_left / WHEEL_R * 60.0f / (2.0f * PI);
  float rpm_right = v_right / WHEEL_R * 60.0f / (2.0f * PI);

  *l_rpm = static_cast<int32_t>(rpm_left);
  *r_rpm = static_cast<int32_t>(rpm_right);
}

void loop() {
}

const char* stateToString(int state) {
    switch (state) {
        case ESTOP:       return "ESTOP";
        case MOTOR_IDLE:  return "MOTOR_IDLE";
        case RF_IDLE:     return "RF_IDLE";
        case DRIVE_READY: return "DRIVE_READY";
        case DRIVE_MANUAL:return "DRIVE_MANUAL";
        case AUTO_IDLE:   return "AUTO_IDLE";
        case DRIVE_AUTO:  return "DRIVE_AUTO";
        case AUTO_FAIL:   return "AUTO_FAIL";
        default:          return "UNKNOWN_STATE";
    }
}
