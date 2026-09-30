#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* 每次只选择一种机器人应用，并重新完整构建固件。 */
#define ROBOT_WHEEL_LEG 0
#define ROBOT_ARM       1

typedef enum
{
    RobotTypeWheelLeg = ROBOT_WHEEL_LEG,
    RobotTypeArm = ROBOT_ARM
} RobotType_e;

#ifndef ROBOT_TYPE
#define ROBOT_TYPE ROBOT_ARM
#endif

#if (ROBOT_TYPE != ROBOT_WHEEL_LEG) && (ROBOT_TYPE != ROBOT_ARM)
#error "ROBOT_TYPE must be ROBOT_WHEEL_LEG or ROBOT_ARM"
#endif

#define ARM_CONTROL_PERIOD_MS       1U
#define ARM_FEEDBACK_TIMEOUT_MS     50U
#define ARM_STARTUP_TIMEOUT_MS      3000U
#define ARM_ENABLE_RETRY_MS         20U

#endif /* 机器人全局配置 */
