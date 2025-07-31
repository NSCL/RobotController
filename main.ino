#define DEBUG

#define ROBOT_DD 1
#define ROBOT_STEERING 2
#define ROBOT_VERSION ROBOT_DD

// Common Includes
// ============================================================
#include "rf_controller.h"
#include "pc_comm.h"
#include "util.h"
#include <Arduino_CAN.h>
#include "mbed.h"

// === Portenta Hardware ===
#define LED_PIN LED_BUILTIN
#define SERIAL_RF Serial3

#ifdef DEBUG
  #define SERIAL_PC Serial1
  #define SERIAL_DEBUG Serial
#else
  #define SERIAL_PC Serial
  #define SERIAL_DEBUG Serial1
#endif

#define PIN_ESTOP 6
#define BATTERY_PIN A0

// === Variables ===
State state = INIT;
uint8_t battery = 140;
int32_t left_rpm_output = 0;
int32_t right_rpm_output = 0;

bool is_estop = false;
bool is_estop_btn_on = false;
bool is_battery_low = false;
PcComm pc_comm;

// Robot Version
// ============================================================
#if ROBOT_VERSION == ROBOT_DD
  // === Robot Hardware ===
  #define WHEEL_R  0.08255
  #define WHEELBASE  0.4

  // === Motor Driver ===
  #include "motor_dd_robot.h"
  #define NODE_ID 1
  ZltechController motor_dd_robot(NODE_ID, WHEEL_R, WHEELBASE);

  // === Robot Command ===
  CommandDDRobot cmd_rf;
  CommandDDRobot cmd_pc;
  CommandDDRobot cmd_input;
  RFControllerDDRobot rf_controller(SERIAL_RF);

  // === RF Controller Channel ===
  #define CH_ESTOP 7
  #define CH_DISCONNECT 5
  #define CH_VELOCITY 1
  #define CH_OMEGA 3
  #define CH_DRIVE_MODE 4

#elif ROBOT_VERSION == ROBOT_STEERING
  // === Robot Hardware ===
  #define WHEEL_R  0.4728 // 120/70 R12
  // === Motor Driver ===
  #include "motor_steer_robot.h"
  const static uint32_t ID_DRIVE_LEFT = (0x0001 << 8) | 120;
  const static uint32_t ID_DRIVE_RIGHT = (0x0001 << 8) | 121;
  const static uint32_t ID_STEER = 0x01;
  MotorSteerRobot motor_steer_robot(ID_DRIVE_LEFT, ID_DRIVE_RIGHT, ID_STEER, WHEEL_R);
  
  // === Robot Command ===
  CommandSteeringRobot cmd_rf;
  CommandSteeringRobot cmd_pc;
  CommandSteeringRobot cmd_input;
  RFControllerSteeringRobot rf_controller(SERIAL_RF);
  
  // === RF Controller Channel ===
  #define CH_ESTOP 7
  #define CH_DISCONNECT 5
  #define CH_VELOCITY 1
  #define CH_STEER 3
  #define CH_GEAR 6
  #define CH_DRIVE_MODE 4

#endif

// ============================================================
// === Threads & Mutex ===
rtos::Thread threadRF; // RF Controller Thread
rtos::Thread threadRobotControl; // Motor Control Thread
rtos::Thread threadPcComm; // PC <-> Portenta Communication Thread
rtos::Mutex dataMutex;

#define FLAG_RF (1UL << 0)
#define FLAG_ROBOT_CONTROL (1UL << 1)
#define FLAG_PC_COMM (1UL << 2)

mbed::Ticker tick1ms;   /* 10 ms*/
uint32_t tick_cnt = 0;
const uint8_t tick_robot_control = 10; // 10 ms
const uint8_t tick_rf = 20; // 20 ms
const uint8_t tick_pc_comm = 10; // 10 ms, why origin 2 ms?
const uint8_t tick_battery = 1000; // 1000 ms
rtos::EventFlags ef;
// ============================================================

void setup() {
  #ifdef DEBUG
    SERIAL_DEBUG.begin(9600);
    delay(1000);
  #endif

  pc_comm.begin(SERIAL_PC);
  delay(50);

  // Motor connection
  state = MOTOR_IDLE;
  #ifdef DEBUG
    SERIAL_DEBUG.println("Start connecting motors...");
  #endif

  #if ROBOT_VERSION == ROBOT_DD
    motor_dd_robot.init();
    motor_dd_robot.setup();
  #elif ROBOT_VERSION == ROBOT_STEERING
    motor_steer_robot.init();
  #endif
  delay(50);

  // RF connection
  state = RF_IDLE;
  #ifdef DEBUG
    SERIAL_DEBUG.println("Start connecting RF");
  #endif

  #if ROBOT_VERSION == ROBOT_DD
    rf_controller.setMaxOmega(20.0);   // [deg]
    rf_controller.setMaxVelocity(1.5); // [m/s]
    rf_controller.setChannel(CH_ESTOP, CH_DISCONNECT, CH_DRIVE_MODE, CH_VELOCITY, CH_OMEGA);          // estop, disconnect, drive_mode, velocity, omega
    rf_controller.begin();
  #elif ROBOT_VERSION == ROBOT_STEERING
    rf_controller.setMaxSteer(20.0);   // [deg]
    rf_controller.setMaxVelocity(2.5); // [m/s]
    rf_controller.setChannel(CH_ESTOP, CH_DISCONNECT, CH_GEAR, CH_DRIVE_MODE, CH_VELOCITY, CH_STEER); // estop, disconnect, gear, drive_mode, velocity, steer
    rf_controller.begin();
  #endif
  
  delay(50);

  state = DRIVE_READY;

  // E-Stop
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ESTOP), detectEstopSW, CHANGE);

  // Thread
  tick1ms.attach(&setFlag, 0.001);            // 1 ms per 1 tick
  threadRF.start(taskRF);                     // Start RF thread
  threadRobotControl.start(taskRobotControl); // Start robot control
  threadPcComm.start(taskPcComm);          // Start PC communication
}

void taskRF(){
  while(true){
    ef.wait_any(FLAG_RF);

    rf_controller.checkConnection();

    if (rf_controller.is_connected) {
      rf_controller.getCommand(cmd_rf);
    }

    #ifdef DEBUG
      #if ROBOT_VERSION == ROBOT_DD
        SERIAL_DEBUG.print("[");
        SERIAL_DEBUG.print(rf_controller.is_connected ? "RECEIVED" : "DISCONNECTED");
        SERIAL_DEBUG.print("] ");
        SERIAL_DEBUG.print("Estop: ");
        SERIAL_DEBUG.print(cmd_rf.estop);
        SERIAL_DEBUG.print(" | Drive Mode: ");
        SERIAL_DEBUG.print(cmd_rf.drive_mode);
        SERIAL_DEBUG.print(" | state: ");
        SERIAL_DEBUG.print(state);
        SERIAL_DEBUG.print(" | Velocity: ");
        SERIAL_DEBUG.print(cmd_rf.velocity);
        SERIAL_DEBUG.print(" | Omega : ");
        SERIAL_DEBUG.print(cmd_rf.omega);
        SERIAL_DEBUG.print(" | State: ");
        SERIAL_DEBUG.print(stateToString(state));
        SERIAL_DEBUG.print(" | Battery: ");
        SERIAL_DEBUG.println(battery);
      #elif ROBOT_VERSION == ROBOT_STEERING
        SERIAL_DEBUG.print("[");
        SERIAL_DEBUG.print(rf_controller.is_connected ? "RECEIVED" : "DISCONNECTED");
        SERIAL_DEBUG.print("] ");
        SERIAL_DEBUG.print("Estop: ");
        SERIAL_DEBUG.print(cmd_rf.estop);
        SERIAL_DEBUG.print(" | Drive Mode: ");
        SERIAL_DEBUG.print(cmd_rf.drive_mode);
        SERIAL_DEBUG.print(" | state: ");
        SERIAL_DEBUG.print(state);
        SERIAL_DEBUG.print(" | Velocity: ");
        SERIAL_DEBUG.print(cmd_rf.velocity);
        SERIAL_DEBUG.print(" | Steer : ");
        SERIAL_DEBUG.print(cmd_rf.steer);
        SERIAL_DEBUG.print(" | Gear : ");
        SERIAL_DEBUG.print(cmd_rf.gear);
        SERIAL_DEBUG.print(" | State: ");
        SERIAL_DEBUG.print(stateToString(state));
        SERIAL_DEBUG.print(" | Battery: ");
        SERIAL_DEBUG.println(battery);
      #endif
    #endif
  }
}

// Motor control 
void taskRobotControl(){
  while(true){
    ef.wait_any(FLAG_ROBOT_CONTROL);

    int32_t l_rpm = 0;
    int32_t r_rpm = 0;

    // Common
    checkBattery();
    is_estop = (is_estop_btn_on || cmd_rf.estop || !rf_controller.is_connected || is_battery_low);

    #if ROBOT_VERSION == ROBOT_DD
      motor_dd_robot.loop();
      if (!motor_dd_robot.is_connected) state = MOTOR_IDLE;
    #elif ROBOT_VERSION == ROBOT_STEERING
      motor_steer_robot.checkConnected();
      if (!motor_steer_robot.is_connected) state = MOTOR_IDLE;
    #endif

    if (is_estop) state = ESTOP;

    #if ROBOT_VERSION == ROBOT_DD
      cmd_input.velocity = 0;
      cmd_input.omega = 0;
      switch (state) {
        case ESTOP: {
          if (!is_estop) {
            state = DRIVE_READY;
            break;
          }
          break;
        }
        case MOTOR_IDLE: {
          if (motor_dd_robot.is_connected) {
            state = DRIVE_READY;
          }
          motor_dd_robot.setup();
          break;
        }
        case RF_IDLE: {
          if (rf_controller.is_connected){
            state = DRIVE_READY;
          }
          break;
        }
        case DRIVE_READY: {
          state = (cmd_rf.drive_mode == MODE_MANUAL) ? DRIVE_MANUAL : AUTO_IDLE;
          break;
        }
        case DRIVE_MANUAL: {
          if (cmd_rf.drive_mode != MODE_MANUAL) {
            state = DRIVE_READY;
            break;
            }
          // Motor Control 
          cmd_input = cmd_rf;
          break;
        }
        case AUTO_IDLE: {
          if (cmd_rf.drive_mode != MODE_AUTO) state = DRIVE_READY;
          state = (pc_comm.is_connected) ? DRIVE_AUTO : AUTO_FAIL;
          break;
        }
        case DRIVE_AUTO: {
          if (cmd_rf.drive_mode != MODE_AUTO) state = DRIVE_READY;
          cmd_input = cmd_pc;
          break;
        }
        case AUTO_FAIL: {
          if (cmd_rf.drive_mode != MODE_AUTO) state = DRIVE_READY;
          if (pc_comm.is_connected) state = DRIVE_AUTO;
          break;
        }
        default: {
          break;
        }
      }
    #elif ROBOT_VERSION == ROBOTROBOT_STEERING_DD
      cmd_input.velocity = 0;
      cmd_input.steer = 0;
      switch (state) {
        case ESTOP: {
          if (!is_estop) {
            state = DRIVE_READY;
            break;
          }
          break;
        }
        case MOTOR_IDLE: {
          if (motor_steer_robot.is_connected) {
            state = DRIVE_READY;
          }
          break;
        }
        case RF_IDLE: {
          if (rf_controller.is_connected){
            state = DRIVE_READY;
          }
          break;
        }
        case DRIVE_READY: {
          state = (cmd_rf.drive_mode == MODE_MANUAL) ? DRIVE_MANUAL : AUTO_IDLE;
          break;
        }
        case DRIVE_MANUAL: {
          if (cmd_rf.drive_mode != MODE_MANUAL) {
            state = DRIVE_READY;
            break;
            }
          // Motor Control 
          cmd_input = cmd_rf;
          break;
        }
        case AUTO_IDLE: {
          if (cmd_rf.drive_mode != MODE_AUTO) state = DRIVE_READY;

          state = (pc_comm.is_connected) ? DRIVE_AUTO : AUTO_FAIL;

          break;
        }
        case DRIVE_AUTO: {
          if (cmd_rf.drive_mode != MODE_AUTO) state = DRIVE_READY;
          cmd_input = cmd_pc;
          break;
        }
        case AUTO_FAIL: {
          if (cmd_rf.drive_mode != MODE_AUTO) state = DRIVE_READY;
          if (pc_comm.is_connected) state = DRIVE_AUTO;
          break;
        }
        default: {
          break;
        }
      }
    #endif
    
    // DRIVE
    #if ROBOT_VERSION == ROBOT_DD
      motor_dd_robot.sendCommand(cmd_input);
      motor_dd_robot.readVelocity(&left_rpm_output, &right_rpm_output);
    #elif ROBOT_VERSION == ROBOT_STEERING
      motor_steer_robot.sendCommand(cmd_input);
    #endif
  }
}

void taskPcComm(){
  while(true){
    ef.wait_any(FLAG_PC_COMM);

    pc_comm.loop();

    #if ROBOT_VERSION == ROBOT_DD
      pc_comm.sendPacket(cmd_rf.drive_mode, cmd_rf.estop, 1, cmd_rf.velocity, cmd_rf.omega, 0, left_rpm_output, right_rpm_output, battery);
      if (pc_comm.is_connected) {
        pc_comm.getCommandDD(cmd_pc);   
      }
      else {
        cmd_pc.velocity = 0;
        cmd_pc.omega = 0;
      }
      
    #elif ROBOT_VERSION == ROBOT_STEERING
      pc_comm.sendPacket(cmd_rf.drive_mode, cmd_rf.estop, cmd_rf.gear, cmd_rf.velocity, cmd_rf.steer, cmd_rf.brake, left_rpm_output, right_rpm_output, battery);
      if (pc_comm.is_connected) {
        pc_comm.getCommandSteer(cmd_pc);   
      }
      else {
        cmd_pc.velocity = 0;
        cmd_pc.steer = 0;
        cmd_pc.gear = GEAR_NEUTRAL;
      }
    #endif    
  }
}

void checkBattery(){
  uint16_t battery_ = analogRead(BATTERY_PIN);
  battery = (battery_ * 310UL) / 1023 - 100;
  is_battery_low = (battery < 120);
}

void setFlag(){
  if(tick_cnt % tick_rf == 0) ef.set(FLAG_RF);
  if(tick_cnt % tick_robot_control == 0) ef.set(FLAG_ROBOT_CONTROL);
  if(tick_cnt % tick_pc_comm == 0) ef.set(FLAG_PC_COMM);
  ++tick_cnt;
}

void detectEstopSW() {
  is_estop_btn_on = (digitalRead(PIN_ESTOP) == LOW);
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
