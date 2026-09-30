#ifndef ARM_CONFIG_H
#define ARM_CONFIG_H

#include <stdint.h>

#include "arm_algorithm.h"
#include "motor.h"

#define ARM_MODEL_NONE  0U
#define ARM_MODEL_3DOF 1U

#ifndef ARM_MODEL_TYPE
#define ARM_MODEL_TYPE ARM_MODEL_3DOF
#endif

typedef struct
{
    MotorType_e motor_type;
    MotorControlMode_e control_mode;

    uint8_t can_bus;
    uint8_t id;

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
} ArmJointConfig_s;

extern const ArmJointConfig_s arm_3dof_joint_config[ARM_JOINT_COUNT];

#endif /* 机械臂配置接口 */
