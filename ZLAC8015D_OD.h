#ifndef ZLAC8015D_OD_H
#define ZLAC8015D_OD_H

// ========== 제어 명령 ==========
#define CONTROLWORD               0x6040
#define CONTROLWORD_BITS          16
#define STATUSWORD                0x6041
#define STATUSWORD_BITS           16
#define MODES_OF_OPERATION        0x6060
#define MODES_OF_OPERATION_BITS   8
#define MODE_OF_OPERATION_DISP    0x6061
#define MODE_OF_OPERATION_DISP_BITS 8
#define ASYN_SYN_CONTROL          0x200F
#define ASYN_CONTROL              0x00
#define SYN_CONTROL               0x01
#define ASYN_SYN_CONTROL_BITS     16

// ========== 속도 관련 ==========
#define Target_velocity                   0x60FF
#define Left_Motor_Target_velocity        0x01
#define Right_Motor_Target_velocity       0x02
#define Target_velocity_Bits              32

#define Velocity_actual_value             0x606C
#define Left_Motor_Velocity_actual_value  0x01
#define Right_Motor_Velocity_actual_value 0x02
#define Velocity_actual_value_Bits        32

// ========== 위치 관련 ==========
#define TARGET_POSITION           0x607A
#define TARGET_POSITION_BITS      32
#define ACTUAL_POSITION           0x6064
#define ACTUAL_POSITION_BITS      32

// ========== 토크 관련 ==========
#define TARGET_TORQUE             0x6071
#define TARGET_TORQUE_BITS        16
#define ACTUAL_TORQUE             0x6077
#define ACTUAL_TORQUE_BITS        16

// ========== 가감속 관련 ==========
#define PROFILE_ACCELERATION         0x6083
#define PROFILE_ACCELERATION_BITS   32
#define PROFILE_DECELERATION         0x6084
#define PROFILE_DECELERATION_BITS   32
#define QUICKSTOP_DECELERATION       0x6085
#define QUICKSTOP_DECELERATION_BITS 32

// ========== 노드 및 통신 ==========
#define CAN_NODE_ID               0x200A
#define CAN_NODE_ID_BITS          8
#define CAN_BAUDRATE              0x200B
#define CAN_BAUDRATE_BITS         8
#define HEARTBEAT_TIME            0x1017
#define HEARTBEAT_TIME_BITS       16

// ========== 사용자 정의 값 ==========
#define MAX_SPEED                 0x2008
#define MAX_SPEED_BITS            16
#define LEFT_MOTOR_ENCODER_LINE   0x200E
#define LEFT_MOTOR_ENCODER_LINE_BITS 16
#define RIGHT_MOTOR_ENCODER_LINE  0x200E
#define RIGHT_MOTOR_ENCODER_LINE_BITS 16

#endif