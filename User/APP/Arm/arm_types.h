#ifndef ARM_TYPES_H
#define ARM_TYPES_H

#include <stdint.h>

#include "arm_app.h"
#include "motor.h"

typedef enum
{
    ArmModeDisabled = 0,
    ArmModeNormal,
    ArmModeCommissioning
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

    uint8_t installed;
    uint8_t output_enabled;
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
    ArmFeedback feedback;

    uint32_t update_count;
} Arm_s;

#endif /* ARM_TYPES_H */
