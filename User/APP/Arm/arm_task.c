#include "arm_task.h"

#include <string.h>

#include "arm_config.h"
#include "arm_limit.h"
#include "cmsis_os.h"
#include "motor_manager.h"
#include "robot_config.h"

#if ROBOT_TYPE == ROBOT_ARM

#define ARM_JOINT0_INDEX                 0U
#define ARM_JOINT0_TARGET_VELOCITY_RAD_S 6.28318531f
#define DM_MOTOR_STATE_ENABLED           1U
#define DM_MOTOR_STATE_FAULT_MIN         8U

/* 机械臂公共对象持有三个关节，具体电机对象由本任务文件统一管理。 */
Arm_s arm;
static Motor_s arm_motor[ARM_JOINT_COUNT];

/* 状态机时间戳和一次性控制标志。 */
static uint32_t arm_start_ms;
static uint32_t arm_last_enable_ms;
static uint8_t arm_initial_reference_captured;
static uint8_t arm_disable_sent;

static float Arm_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Arm_JointToMotorPosition(const ArmJoint_s *joint,
                                      float joint_position)
{
    /* 关节坐标转换为电机坐标。 */
    return joint->direction * joint_position + joint->zero_offset;
}

static float Arm_MotorToJointPosition(const ArmJoint_s *joint,
                                      float motor_position)
{
    /* 电机反馈坐标转换为关节坐标。 */
    return joint->direction * (motor_position - joint->zero_offset);
}

static void Arm_RequestEnable(uint32_t now)
{
    uint8_t joint;

    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        ArmJoint_s *arm_joint = &arm.joint[joint];

        /* output_enabled 是唯一使能门控，当前只有 Joint0 满足条件。 */
        if ((arm_joint->installed != 0U) &&
            (arm_joint->output_enabled != 0U) &&
            (arm_joint->motor != 0))
        {
            Motor_Enable(arm_joint->motor);
            arm.enable_request_count++;
        }
    }
    arm_last_enable_ms = now;
}

static void Arm_DisableOutput(void)
{
    uint8_t joint;

    /* 同一故障只发送一次失能帧，避免在 1 kHz 任务中重复发送。 */
    if (arm_disable_sent != 0U)
    {
        return;
    }

    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        ArmJoint_s *arm_joint = &arm.joint[joint];

        arm_joint->output_allowed = 0U;
        if ((arm_joint->installed != 0U) &&
            (arm_joint->output_enabled != 0U) &&
            (arm_joint->motor != 0))
        {
            Motor_Disable(arm_joint->motor);
        }
    }
    arm_disable_sent = 1U;
}

static void Arm_UpdateState(void)
{
    uint8_t joint;
    uint32_t now = HAL_GetTick();

    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        ArmJoint_s *arm_joint = &arm.joint[joint];
        Motor_s *motor = arm_joint->motor;

        if (motor == 0)
        {
            arm.feedback.online[joint] = 0U;
            continue;
        }

        /* 反馈超时只更新在线状态，失能操作由异常处理阶段统一执行。 */
        if ((motor->last_feedback_ms == 0U) ||
            ((now - motor->last_feedback_ms) > ARM_FEEDBACK_TIMEOUT_MS))
        {
            motor->online = 0U;
        }

        /* 三个关节使用同一套坐标变换和反馈拷贝逻辑。 */
        arm.feedback.q[joint] =
            Arm_MotorToJointPosition(arm_joint, motor->feedback.position);
        arm.feedback.dq[joint] =
            arm_joint->direction * motor->feedback.velocity;
        arm.feedback.torque[joint] =
            arm_joint->direction * motor->feedback.torque;
        arm.feedback.online[joint] = motor->online;
        arm.feedback.motor_state[joint] = motor->feedback.state;
    }
}

static void Arm_HandleException(void)
{
    uint8_t joint;
    uint32_t now = HAL_GetTick();

    /* 人工停止请求优先级最高，锁存后不再自动重新使能。 */
    if (arm.stop_requested != 0U)
    {
        arm.state = ArmStateDisabled;
        Arm_DisableOutput();
        return;
    }

    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        ArmJoint_s *arm_joint = &arm.joint[joint];
        Motor_s *motor = arm_joint->motor;

        /* Joint1、Joint2 只观察反馈，不阻塞 Joint0 单轴验证。 */
        if ((arm_joint->installed == 0U) ||
            (arm_joint->output_enabled == 0U))
        {
            continue;
        }

        if ((motor == 0) || (motor->initialized == 0U) ||
            (motor->fault != 0U) ||
            (motor->feedback.state >= DM_MOTOR_STATE_FAULT_MIN))
        {
            arm.state = ArmStateFault;
            Arm_DisableOutput();
            return;
        }

        /* 未在线或未使能时按固定周期重发 Joint0 使能请求。 */
        if ((motor->online == 0U) ||
            (motor->feedback.state != DM_MOTOR_STATE_ENABLED))
        {
            if ((now - arm_start_ms) >= ARM_STARTUP_TIMEOUT_MS)
            {
                arm.state = ArmStateFault;
                Arm_DisableOutput();
                return;
            }

            arm.state = ArmStateWaitingFeedback;
            if ((now - arm_last_enable_ms) >= ARM_ENABLE_RETRY_MS)
            {
                Arm_RequestEnable(now);
            }
            return;
        }
    }

    /* 首次有效反馈作为初始位置目标，避免默认位置 0 导致突然回零。 */
    if (arm_initial_reference_captured == 0U)
    {
        for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
        {
            if (arm.feedback.online[joint] != 0U)
            {
                arm.command.q[joint] = arm.feedback.q[joint];
            }
        }
        arm_initial_reference_captured = 1U;
    }

    arm_disable_sent = 0U;
    arm.state = ArmStateRunning;
}

static void Arm_SetMode(void)
{
    /* 只有正常运行状态进入控制模式，其余状态统一禁止输出。 */
    arm.mode = (arm.state == ArmStateRunning) ?
               ArmModeNormal : ArmModeDisabled;
}

static void Arm_SetReference(void)
{
    uint8_t joint;

    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        ArmJoint_s *arm_joint = &arm.joint[joint];

        /* 正式命令先经过机械范围、速度和力矩限制再成为参考量。 */
        arm.reference.q[joint] =
            ArmLimit_Clamp(arm.command.q[joint],
                           arm_joint->min_position,
                           arm_joint->max_position);
        arm.reference.dq[joint] =
            ArmLimit_Clamp(arm.command.dq[joint],
                           -arm_joint->max_velocity,
                           arm_joint->max_velocity);
        arm.reference.torque[joint] =
            ArmLimit_Clamp(arm.command.torque[joint],
                           -arm_joint->max_torque,
                           arm_joint->max_torque);
    }
}

static void Arm_Calculate(void)
{
    uint8_t joint;

    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        ArmJoint_s *arm_joint = &arm.joint[joint];
        Motor_s *motor = arm_joint->motor;

        if (motor == 0)
        {
            continue;
        }

        /* 三个关节共用计算路径，实际发送许可由安全阶段决定。 */
        motor->command.position =
            Arm_JointToMotorPosition(arm_joint, arm.reference.q[joint]);
        motor->command.velocity =
            arm_joint->direction * arm.reference.dq[joint];
        motor->command.torque =
            arm_joint->direction * arm.reference.torque[joint];
        motor->command.kp = arm_joint->kp;
        motor->command.kd = arm_joint->kd;
    }
}

static void Arm_Safety(void)
{
    uint8_t joint;

    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        ArmJoint_s *arm_joint = &arm.joint[joint];
        Motor_s *motor = arm_joint->motor;
        float velocity_error;
        float torque_budget;

        /* 每个周期默认关闭输出，通过全部安全检查后再放行。 */
        arm_joint->output_allowed = 0U;

        if ((arm_joint->installed == 0U) ||
            (arm_joint->output_enabled == 0U) ||
            (arm.state != ArmStateRunning) ||
            (motor == 0) || (motor->initialized == 0U) ||
            (motor->online == 0U) || (motor->fault != 0U))
        {
            continue;
        }

        /* 到达机械边界时禁止继续向外运动，但允许反向退出。 */
        if (((arm.feedback.q[joint] >= arm_joint->max_position) &&
             (arm.reference.dq[joint] > 0.0f)) ||
            ((arm.feedback.q[joint] <= arm_joint->min_position) &&
             (arm.reference.dq[joint] < 0.0f)))
        {
            motor->command.velocity = 0.0f;
        }

        motor->command.velocity =
            ArmLimit_Clamp(motor->command.velocity,
                           -arm_joint->max_velocity,
                           arm_joint->max_velocity);
        motor->command.torque =
            ArmLimit_Clamp(motor->command.torque,
                           -arm_joint->max_torque,
                           arm_joint->max_torque);

        /* 动态收紧 Kd，保证阻尼项不超过额定力矩预算。 */
        torque_budget = arm_joint->max_torque - Arm_Abs(motor->command.torque);
        velocity_error = motor->command.velocity - motor->feedback.velocity;
        if (torque_budget <= 0.0f)
        {
            motor->command.kd = 0.0f;
        }
        else if ((Arm_Abs(velocity_error) > 0.0f) &&
                 ((motor->command.kd * Arm_Abs(velocity_error)) > torque_budget))
        {
            motor->command.kd = torque_budget / Arm_Abs(velocity_error);
        }

        arm_joint->output_allowed = 1U;
    }
}

static void Arm_SendCommand(void)
{
    uint8_t joint;

    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        ArmJoint_s *arm_joint = &arm.joint[joint];

        /* 配置门控和本周期安全许可必须同时成立。 */
        if ((arm_joint->installed == 0U) ||
            (arm_joint->output_enabled == 0U) ||
            (arm_joint->output_allowed == 0U) ||
            (arm_joint->motor == 0))
        {
            continue;
        }

        Motor_SendCommand(arm_joint->motor);
    }
}

void ArmTask_Init(void)
{
    uint8_t joint;
    uint32_t now;

    memset(&arm, 0, sizeof(arm));
    MotorManager_Init();
    arm.state = ArmStateWaitingFeedback;
    arm.mode = ArmModeDisabled;

    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        const ArmJointConfig_s *config = &arm_3dof_joint_config[joint];
        ArmJoint_s *arm_joint = &arm.joint[joint];

        arm_joint->direction = config->direction;
        arm_joint->zero_offset = config->zero_offset;
        arm_joint->min_position = config->min_position;
        arm_joint->max_position = config->max_position;
        arm_joint->max_velocity = config->max_velocity;
        arm_joint->max_torque = config->max_torque;
        arm_joint->kp = config->kp;
        arm_joint->kd = config->kd;
        arm_joint->installed = config->installed;
        arm_joint->output_enabled = config->output_enabled;
        arm_joint->motor = &arm_motor[joint];

        /* 与参考工程一致，每个电机只调用一次统一 MotorInit。 */
        if ((config->installed != 0U) &&
            (MotorInit(arm_joint->motor,
                       config->id,
                       config->can_bus,
                       config->motor_type,
                       config->control_mode) != MOTOR_STATUS_OK))
        {
            arm_joint->motor = 0;
            arm.state = ArmStateFault;
            return;
        }
    }

    /* 三个电机必须全部由 MotorInit 自动登记成功。 */
    if (MotorManager_GetRegisteredCount() != ARM_JOINT_COUNT)
    {
        arm.state = ArmStateFault;
        return;
    }

    /* 用户指定的初始正式命令：Joint0 以每秒一圈的速度旋转。 */
    arm.command.dq[ARM_JOINT0_INDEX] = ARM_JOINT0_TARGET_VELOCITY_RAD_S;

    now = HAL_GetTick();
    arm_start_ms = now;
    arm_last_enable_ms = now;
    arm_initial_reference_captured = 0U;
    arm_disable_sent = 0U;
    Arm_RequestEnable(now);
}

void ArmTask_Update(void)
{
    /* 固定控制流水线，所有阶段均在本文件中实现。 */
    Arm_UpdateState();
    Arm_HandleException();
    Arm_SetMode();
    Arm_SetReference();
    Arm_Calculate();
    Arm_Safety();
    Arm_SendCommand();
    arm.update_count++;
}

void ArmTask_Run(void const *argument)
{
    (void)argument;
    ArmTask_Init();

    /* 机械臂控制周期由全局配置统一管理。 */
    for (;;)
    {
        ArmTask_Update();
        osDelay(ARM_CONTROL_PERIOD_MS);
    }
}

void Arm_SetCommand(const ArmCommand *command)
{
    /* 外部模块通过完整命令对象更新三关节目标。 */
    if (command != 0)
    {
        memcpy(&arm.command, command, sizeof(arm.command));
    }
}

void Arm_SetJointTarget(uint8_t joint, float position)
{
    /* 单关节位置目标最终仍需经过统一参考量和安全限制。 */
    if (joint < ARM_JOINT_COUNT)
    {
        arm.command.q[joint] = position;
    }
}

void Arm_SetJointVelocity(uint8_t joint, float velocity)
{
    /* 单关节速度目标最终仍需经过统一速度和力矩限制。 */
    if (joint < ARM_JOINT_COUNT)
    {
        arm.command.dq[joint] = velocity;
    }
}

void Arm_Disable(void)
{
    /* 锁存人工停止请求，防止下一周期自动重新使能。 */
    arm.stop_requested = 1U;
    arm.state = ArmStateDisabled;
    arm.mode = ArmModeDisabled;
    arm_disable_sent = 0U;
    Arm_DisableOutput();
}

ArmState_e Arm_GetState(void)
{
    return arm.state;
}

#endif /* 当前构建为机械臂 */
