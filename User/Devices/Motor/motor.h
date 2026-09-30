#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#include "dm4310_drv.h"

typedef enum
{
    MOTOR_TYPE_NONE = 0,
    MOTOR_TYPE_DM4310,
    MOTOR_TYPE_DJI_M3508,
    MOTOR_TYPE_DJI_M2006,
    MOTOR_TYPE_DJI_GM6020
} MotorType_e;

typedef enum
{
    MOTOR_MODE_DISABLED = 0,
    MOTOR_MODE_MIT,
    MOTOR_MODE_POSITION,
    MOTOR_MODE_SPEED,
    MOTOR_MODE_TORQUE,
    MOTOR_MODE_CURRENT
} MotorControlMode_e;

typedef enum
{
    MOTOR_PROTOCOL_NONE = 0,
    MOTOR_PROTOCOL_DM,
    MOTOR_PROTOCOL_DJI
} MotorProtocol_e;

typedef enum
{
    MOTOR_STATUS_OK = 0,
    MOTOR_STATUS_INVALID_ARGUMENT,
    MOTOR_STATUS_UNSUPPORTED,
    MOTOR_STATUS_REGISTRATION_FAILED
} MotorStatus_e;

typedef struct
{
    float position;
    float velocity;
    float torque;
    float current;
    uint8_t state;
} MotorFeedback_s;

typedef struct
{
    float position;
    float velocity;
    float torque;
    float current;
    float kp;
    float kd;
} MotorCommand_s;

typedef union
{
    Joint_Motor_t dm4310;
} MotorDriverData_u;

typedef struct Motor_s
{
    uint8_t id;
    MotorType_e type;
    MotorControlMode_e control_mode;
    uint8_t can_bus;

    uint8_t initialized;
    uint8_t online;
    uint8_t fault;
    uint32_t last_feedback_ms;

    MotorFeedback_s feedback;
    MotorCommand_s command;
    MotorDriverData_u driver;
} Motor_s;

MotorProtocol_e Motor_GetProtocol(MotorType_e type);
MotorStatus_e MotorInit(Motor_s *motor,
                        uint8_t id,
                        uint8_t can_bus,
                        MotorType_e type,
                        MotorControlMode_e control_mode);
void Motor_Enable(Motor_s *motor);
void Motor_Disable(Motor_s *motor);
void Motor_SendCommand(Motor_s *motor);
void Motor_DecodeFeedback(Motor_s *motor,
                          uint8_t *data,
                          uint32_t data_len);

#endif /* MOTOR_H */
