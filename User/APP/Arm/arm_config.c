#include "arm_config.h"

#include "robot_config.h"

#if ROBOT_TYPE == ROBOT_ARM

/* DM-J4310-2EC V1.2（24 V）：空载 200 rpm，额定 3.5 N·m，峰值 12.5 N·m。 */
#define DM4310_V12_MAX_VELOCITY_RAD_S 20.9439510f
#define DM4310_V12_RATED_TORQUE_NM     3.5f
#define DM4310_V12_VELOCITY_KD         4.0f

const ArmJointConfig_s arm_3dof_joint_config[ARM_JOINT_COUNT] =
{
    {
        MOTOR_TYPE_DM4310,
        MOTOR_MODE_MIT,
        1U,
        1U,
        1.0f,
        0.0f,
        -4.71238898f,
        4.71238898f,
        DM4310_V12_MAX_VELOCITY_RAD_S,
        DM4310_V12_RATED_TORQUE_NM,
        0.0f,
        DM4310_V12_VELOCITY_KD,
        1U,
        1U
    },
    {
        MOTOR_TYPE_DM4310,
        MOTOR_MODE_MIT,
        1U,
        2U,
        1.0f,
        0.0f,
        -1.10f,
        1.20f,
        DM4310_V12_MAX_VELOCITY_RAD_S,
        DM4310_V12_RATED_TORQUE_NM,
        0.0f,
        DM4310_V12_VELOCITY_KD,
        1U,
        0U
    },
    {
        MOTOR_TYPE_DM4310,
        MOTOR_MODE_MIT,
        1U,
        3U,
        1.0f,
        0.0f,
        -2.50f,
        0.0f,
        DM4310_V12_MAX_VELOCITY_RAD_S,
        DM4310_V12_RATED_TORQUE_NM,
        0.0f,
        DM4310_V12_VELOCITY_KD,
        1U,
        0U
    }
};

#undef DM4310_V12_MAX_VELOCITY_RAD_S
#undef DM4310_V12_RATED_TORQUE_NM
#undef DM4310_V12_VELOCITY_KD

#endif /* 当前构建为机械臂 */
