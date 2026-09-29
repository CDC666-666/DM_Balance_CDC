#include "arm_app.h"

#include <string.h>

#include "arm_limit.h"
#include "cmsis_os.h"
#include "dm4310_drv.h"
#include "fdcan.h"
#include "robot_config.h"

#define ARM_JOINT2_INDEX 2U
#define DM_MOTOR_STATE_DISABLED 0U
#define DM_MOTOR_STATE_ENABLED  1U
#define DM_MOTOR_STATE_FAULT_MIN 8U

static const ArmJointConfig arm_joint_config[ARM_JOINT_COUNT] =
{
    {0U, 0U, ARM_JOINT2_MOTOR_VERSION, 0U, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    {0U, 0U, ARM_JOINT2_MOTOR_VERSION, 0U, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    {
        ARM_JOINT2_MOTOR_ID,
        ARM_JOINT2_MASTER_ID,
        ARM_JOINT2_MOTOR_VERSION,
        ARM_JOINT2_CAN_BUS,
        ARM_JOINT2_DIRECTION,
        ARM_JOINT2_ZERO_OFFSET,
        ARM_JOINT2_MIN_POSITION,
        ARM_JOINT2_MAX_POSITION,
        ARM_JOINT2_MAX_VELOCITY,
        ARM_JOINT2_MAX_TORQUE
    }
};

#if !ARM_JOINT2_SPEED_TEST_ENABLE
static const float arm_min_position[ARM_JOINT_COUNT] =
{
    0.0f, 0.0f, ARM_JOINT2_MIN_POSITION
};

static const float arm_max_position[ARM_JOINT_COUNT] =
{
    0.0f, 0.0f, ARM_JOINT2_MAX_POSITION
};

static const float arm_max_velocity[ARM_JOINT_COUNT] =
{
    0.0f, 0.0f, ARM_JOINT2_MAX_VELOCITY
};
#endif

static Joint_Motor_t arm_joint2_motor;
#if !ARM_JOINT2_SPEED_TEST_ENABLE
static ArmAlgorithm_t arm_algorithm;
#endif
static ArmCommand arm_command;
static volatile uint32_t arm_last_feedback_tick;
static volatile uint8_t arm_feedback_seen;
static uint32_t arm_start_tick;
static uint32_t arm_last_enable_tick;
#if !ARM_JOINT2_SPEED_TEST_ENABLE
static uint32_t arm_test_start_tick;
#endif
static ArmState_e arm_state = ArmStateDisabled;
ArmDebugInfo arm_debug;

static void Arm_SetState(ArmState_e state)
{
    arm_state = state;
    arm_debug.app_state = (uint8_t)state;
}

static hcan_t *Arm_GetCan(void)
{
#if ARM_JOINT2_CAN_BUS == 1U
    return &hfdcan1;
#else
    return &hfdcan2;
#endif
}

#if !ARM_JOINT2_SPEED_TEST_ENABLE
static float Arm_JointToMotorPosition(float joint_position)
{
    return arm_joint_config[ARM_JOINT2_INDEX].direction * joint_position +
           arm_joint_config[ARM_JOINT2_INDEX].zero_offset;
}

static float Arm_MotorToJointPosition(float motor_position)
{
    return arm_joint_config[ARM_JOINT2_INDEX].direction *
           (motor_position - arm_joint_config[ARM_JOINT2_INDEX].zero_offset);
}
#endif

static void Arm_SendEnable(void)
{
    arm_debug.enable_request_count++;
    enable_motor_mode(Arm_GetCan(),
                      arm_joint_config[ARM_JOINT2_INDEX].motor_id,
                      MIT_MODE);
}

static void Arm_SendDisable(void)
{
    disable_motor_mode(Arm_GetCan(),
                       arm_joint_config[ARM_JOINT2_INDEX].motor_id,
                       MIT_MODE);
}

#if !ARM_JOINT2_SPEED_TEST_ENABLE
static void Arm_UpdateLocalTest(uint32_t elapsed)
{
#if ARM_LOCAL_TEST_ENABLE
    uint32_t step = elapsed / ARM_LOCAL_TEST_STEP_MS;

    switch (step)
    {
        case 0U: arm_command.q[ARM_JOINT2_INDEX] = 0.0f; break;
        case 1U: arm_command.q[ARM_JOINT2_INDEX] = 0.2f; break;
        case 2U: arm_command.q[ARM_JOINT2_INDEX] = 0.0f; break;
        case 3U: arm_command.q[ARM_JOINT2_INDEX] = -0.2f; break;
        default: arm_command.q[ARM_JOINT2_INDEX] = 0.0f; break;
    }
#else
    (void)elapsed;
#endif
}
#endif

static void Arm_RunContinuousSpeed(void)
{
    float target_velocity = arm_joint_config[ARM_JOINT2_INDEX].direction *
                            ARM_JOINT2_SPEED_RAD_S;
    float velocity_error = target_velocity - arm_joint2_motor.para.vel;
    float velocity_error_abs = (velocity_error < 0.0f) ?
                               -velocity_error : velocity_error;
    float kd = ARM_JOINT2_SPEED_MIT_KD;

    /* Keep the MIT damping term within the configured torque budget. */
    if ((velocity_error_abs > 0.0f) &&
        ((kd * velocity_error_abs) > arm_joint_config[ARM_JOINT2_INDEX].max_torque))
    {
        kd = arm_joint_config[ARM_JOINT2_INDEX].max_torque / velocity_error_abs;
    }

    mit_ctrl(Arm_GetCan(),
             arm_joint_config[ARM_JOINT2_INDEX].motor_id,
             0.0f,
             target_velocity,
             0.0f,
             kd,
             0.0f);
}

void Arm_Init(void)
{
    uint32_t now = HAL_GetTick();

    memset(&arm_joint2_motor, 0, sizeof(arm_joint2_motor));
    memset(&arm_command, 0, sizeof(arm_command));
    memset(&arm_debug, 0, sizeof(arm_debug));
    joint_motor_init(&arm_joint2_motor,
                     arm_joint_config[ARM_JOINT2_INDEX].motor_id,
                     MIT_MODE);

    arm_feedback_seen = 0U;
    arm_last_feedback_tick = now;
    arm_start_tick = now;
    arm_last_enable_tick = now;
    Arm_SetState(ArmStateWaitingFeedback);
    Arm_SendEnable();
}

void Arm_Update(void)
{
    uint32_t now = HAL_GetTick();
#if !ARM_JOINT2_SPEED_TEST_ENABLE
    float initial_position[ARM_JOINT_COUNT] = {0.0f, 0.0f, 0.0f};
    float desired_motor_position;
    float limited_motor_position;
    float torque;
    float torque_abs;
    float torque_budget;
    float velocity_abs;
    float kd;
#endif

    arm_debug.update_count++;

    if (arm_state == ArmStateWaitingFeedback)
    {
        if ((arm_feedback_seen != 0U) &&
            (arm_joint2_motor.para.state == DM_MOTOR_STATE_ENABLED))
        {
#if !ARM_JOINT2_SPEED_TEST_ENABLE
            initial_position[ARM_JOINT2_INDEX] = Arm_MotorToJointPosition(arm_joint2_motor.para.pos);
            initial_position[ARM_JOINT2_INDEX] = ArmLimit_Clamp(
                initial_position[ARM_JOINT2_INDEX],
                arm_joint_config[ARM_JOINT2_INDEX].min_position,
                arm_joint_config[ARM_JOINT2_INDEX].max_position);
            ArmAlgorithm_Init(&arm_algorithm, initial_position);
            memcpy(arm_command.q, initial_position, sizeof(initial_position));
            arm_test_start_tick = now;
#endif
            Arm_SetState(ArmStateRunning);
        }
        else if ((arm_feedback_seen != 0U) &&
                 (arm_joint2_motor.para.state >= DM_MOTOR_STATE_FAULT_MIN))
        {
            Arm_SetState(ArmStateFault);
        }
        else if ((arm_feedback_seen == 0U) &&
                 ((now - arm_start_tick) >= ARM_STARTUP_TIMEOUT_MS))
        {
            Arm_SendDisable();
            Arm_SetState(ArmStateFault);
        }
        else if ((now - arm_last_enable_tick) >= ARM_ENABLE_RETRY_MS)
        {
            Arm_SendEnable();
            arm_last_enable_tick = now;
        }
        return;
    }

    if (arm_state != ArmStateRunning)
    {
        return;
    }

    if ((now - arm_last_feedback_tick) > ARM_FEEDBACK_TIMEOUT_MS)
    {
        Arm_SendDisable();
        Arm_SetState(ArmStateFault);
        return;
    }

    if (arm_joint2_motor.para.state >= DM_MOTOR_STATE_FAULT_MIN)
    {
        Arm_SetState(ArmStateFault);
        return;
    }

    if (arm_joint2_motor.para.state != DM_MOTOR_STATE_ENABLED)
    {
        arm_start_tick = now;
        arm_last_enable_tick = now;
        Arm_SetState(ArmStateWaitingFeedback);
        Arm_SendEnable();
        return;
    }

#if ARM_JOINT2_SPEED_TEST_ENABLE
    Arm_RunContinuousSpeed();
    return;
#else
    Arm_UpdateLocalTest(now - arm_test_start_tick);
    ArmAlgorithm_SetTarget(&arm_algorithm, arm_command.q);
    ArmAlgorithm_Update(&arm_algorithm,
                        arm_min_position,
                        arm_max_position,
                        arm_max_velocity,
                        ((float)ARM_CONTROL_PERIOD_MS) / 1000.0f);

    torque = ArmLimit_Clamp(
        arm_command.torque[ARM_JOINT2_INDEX],
        -arm_joint_config[ARM_JOINT2_INDEX].max_torque,
        arm_joint_config[ARM_JOINT2_INDEX].max_torque);
    torque_abs = (torque < 0.0f) ? -torque : torque;
    torque_budget = arm_joint_config[ARM_JOINT2_INDEX].max_torque - torque_abs;
    velocity_abs = (arm_joint2_motor.para.vel < 0.0f) ?
                   -arm_joint2_motor.para.vel : arm_joint2_motor.para.vel;
    kd = ARM_JOINT2_MIT_KD;
    if ((velocity_abs > 0.0f) && ((kd * velocity_abs) > torque_budget))
    {
        kd = torque_budget / velocity_abs;
    }

    desired_motor_position = Arm_JointToMotorPosition(
        arm_algorithm.joint_output[ARM_JOINT2_INDEX]);
    limited_motor_position = ArmLimit_MitPosition(
        desired_motor_position,
        arm_joint2_motor.para.pos,
        arm_joint2_motor.para.vel,
        ARM_JOINT2_MIT_KP,
        kd,
        torque_budget);

    mit_ctrl(Arm_GetCan(),
             arm_joint_config[ARM_JOINT2_INDEX].motor_id,
             limited_motor_position,
             0.0f,
             ARM_JOINT2_MIT_KP,
             kd,
             arm_joint_config[ARM_JOINT2_INDEX].direction * torque);
#endif
}

void Arm_SetJointTarget(uint8_t joint, float position)
{
    if (joint < ARM_JOINT_COUNT)
    {
        arm_command.q[joint] = position;
    }
}

void Arm_SetCommand(const ArmCommand *command)
{
    if (command != 0)
    {
        arm_command = *command;
    }
}

void Arm_Stop(void)
{
    Arm_Disable();
}

void Arm_Disable(void)
{
    Arm_SendDisable();
    Arm_SetState(ArmStateDisabled);
}

ArmState_e Arm_GetState(void)
{
    return arm_state;
}

void Arm_OnCanFeedback(uint8_t can_bus,
                       uint16_t master_id,
                       uint8_t *rx_data,
                       uint32_t data_len)
{
    if ((can_bus == arm_joint_config[ARM_JOINT2_INDEX].can_bus) &&
        (master_id == arm_joint_config[ARM_JOINT2_INDEX].master_id))
    {
        dm4310_fbdata(&arm_joint2_motor, rx_data, data_len);
        arm_debug.feedback_count++;
        arm_debug.last_feedback_id = master_id;
        arm_debug.last_motor_id = (uint8_t)arm_joint2_motor.para.id;
        arm_debug.last_motor_state = (uint8_t)arm_joint2_motor.para.state;
        arm_last_feedback_tick = HAL_GetTick();
        arm_feedback_seen = 1U;
    }
}
