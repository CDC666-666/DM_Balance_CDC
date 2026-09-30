#ifndef ARM_TYPES_H
#define ARM_TYPES_H

#include <stdint.h>

#include "arm_algorithm.h"
#include "motor.h"

typedef struct
{
    float q[ARM_JOINT_COUNT];
    float dq[ARM_JOINT_COUNT];
    float torque[ARM_JOINT_COUNT];
} ArmCommand;

typedef enum
{
    ArmStateDisabled = 0,
    ArmStateWaitingFeedback,
    ArmStateRunning,
    ArmStateFault
} ArmState_e;

typedef enum
{
    ArmModeDisabled = 0,
    ArmModeNormal
} ArmMode_e;

typedef struct
{
    Motor_s *motor;

    float direction;
    float zero_offset;
    float min_position;
    float max_position;
    float max_velocity;
    float max_torque;
    float kp;
    float kd;

    uint8_t installed;
    uint8_t output_enabled;
    uint8_t output_allowed;
} ArmJoint_s;

typedef struct
{
    float q[ARM_JOINT_COUNT];
    float dq[ARM_JOINT_COUNT];
    float torque[ARM_JOINT_COUNT];

    uint8_t online[ARM_JOINT_COUNT];
    uint8_t motor_state[ARM_JOINT_COUNT];
} ArmFeedback;

typedef struct
{
    ArmJoint_s joint[ARM_JOINT_COUNT];

    ArmState_e state;
    ArmMode_e mode;

    ArmCommand command;
    ArmCommand reference;
    ArmFeedback feedback;

    uint32_t update_count;
    uint32_t enable_request_count;
    uint8_t stop_requested;
} Arm_s;

#endif /* 机械臂公共类型 */
