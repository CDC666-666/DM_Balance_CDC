#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* Select exactly one application and rebuild the firmware. */
#define ROBOT_WHEEL_LEG 0
#define ROBOT_ARM       1

typedef enum
{
    RobotTypeWheelLeg = ROBOT_WHEEL_LEG,
    RobotTypeArm = ROBOT_ARM
} RobotType_e;

#ifndef ROBOT_TYPE
#define ROBOT_TYPE ROBOT_WHEEL_LEG
#endif

#if (ROBOT_TYPE != ROBOT_WHEEL_LEG) && (ROBOT_TYPE != ROBOT_ARM)
#error "ROBOT_TYPE must be ROBOT_WHEEL_LEG or ROBOT_ARM"
#endif

/*
 * Joint2 first-stage defaults. The CAN IDs are the existing verified
 * FDCAN1 mapping (command ID 8, feedback/master ID 4). Verify them against
 * the actual arm wiring before selecting ROBOT_ARM.
 */
#define ARM_JOINT2_CAN_BUS          1U
#define ARM_JOINT2_MOTOR_ID         8U
#define ARM_JOINT2_MASTER_ID        4U
#define ARM_JOINT2_MOTOR_VERSION    2U
#define ARM_JOINT2_DIRECTION        1.0f
#define ARM_JOINT2_ZERO_OFFSET      0.0f
#define ARM_JOINT2_MIN_POSITION    (-0.25f)
#define ARM_JOINT2_MAX_POSITION     0.25f
#define ARM_JOINT2_MAX_VELOCITY     0.20f
#define ARM_JOINT2_MAX_TORQUE       0.50f
#define ARM_JOINT2_MIT_KP           2.0f
#define ARM_JOINT2_MIT_KD           0.10f

#define ARM_CONTROL_PERIOD_MS       1U
#define ARM_FEEDBACK_TIMEOUT_MS     50U
#define ARM_STARTUP_TIMEOUT_MS      500U
#define ARM_ENABLE_RETRY_MS         20U

/* Set to 1 only after CAN IDs, direction, zero and mechanical clearance are checked. */
#define ARM_LOCAL_TEST_ENABLE       0
#define ARM_LOCAL_TEST_STEP_MS      2000U

#if (ARM_JOINT2_CAN_BUS != 1U) && (ARM_JOINT2_CAN_BUS != 2U)
#error "ARM_JOINT2_CAN_BUS must be 1 or 2"
#endif

#if (ARM_LOCAL_TEST_ENABLE != 0) && (ARM_LOCAL_TEST_ENABLE != 1)
#error "ARM_LOCAL_TEST_ENABLE must be 0 or 1"
#endif

#endif /* ROBOT_CONFIG_H */
