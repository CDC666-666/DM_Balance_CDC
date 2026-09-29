#include "arm_config.h"

#include "robot_config.h"

#if ROBOT_TYPE == ROBOT_ARM

const ArmJointConfig_s arm_3dof_joint_config[ARM_JOINT_COUNT] =
{
    {
        MOTOR_TYPE_NONE,
        MOTOR_MODE_DISABLED,
        0U,
        0U,
        1.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0U,
        0U
    },
    {
        MOTOR_TYPE_NONE,
        MOTOR_MODE_DISABLED,
        0U,
        0U,
        1.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0U,
        0U
    },
    {
        MOTOR_TYPE_DM4310,
        MOTOR_MODE_MIT,
        ARM_JOINT2_CAN_BUS,
        ARM_JOINT2_MOTOR_ID,
        ARM_JOINT2_DIRECTION,
        ARM_JOINT2_ZERO_OFFSET,
        ARM_JOINT2_MIN_POSITION,
        ARM_JOINT2_MAX_POSITION,
        ARM_JOINT2_MAX_VELOCITY,
        ARM_JOINT2_MAX_TORQUE,
        1U,
        0U
    }
};

#endif /* ROBOT_TYPE == ROBOT_ARM */
