#include "motor_dm.h"

#include "fdcan.h"

#define DM_MOTOR_STATE_FAULT_MIN 8U

static hcan_t *MotorDM_GetCan(uint8_t can_bus)
{
    if (can_bus == 1U)
    {
        return &hfdcan1;
    }
    if (can_bus == 2U)
    {
        return &hfdcan2;
    }
    return 0;
}

void MotorDM_DecodeFeedback(Motor_s *motor, uint8_t *data, uint32_t data_len)
{
    Joint_Motor_t *driver;

    if ((motor == 0) || (data == 0) ||
        (motor->initialized == 0U) ||
        (motor->type != MOTOR_TYPE_DM4310) ||
        (data_len != FDCAN_DLC_BYTES_8))
    {
        return;
    }

    driver = &motor->driver.dm4310;
    dm4310_fbdata(driver, data, data_len);

    motor->feedback.position = driver->para.pos;
    motor->feedback.velocity = driver->para.vel;
    motor->feedback.torque = driver->para.tor;
    motor->feedback.current = 0.0f;
    motor->feedback.state = (uint8_t)driver->para.state;
    motor->last_feedback_ms = HAL_GetTick();
    motor->online = 1U;
    motor->fault = (driver->para.state >= DM_MOTOR_STATE_FAULT_MIN) ? 1U : 0U;
}

void MotorDM_Enable(Motor_s *motor)
{
    hcan_t *hcan;

    if ((motor == 0) || (motor->initialized == 0U) ||
        (motor->type != MOTOR_TYPE_DM4310))
    {
        return;
    }

    hcan = MotorDM_GetCan(motor->can_bus);
    if (hcan != 0)
    {
        enable_motor_mode(hcan, motor->id, motor->driver.dm4310.mode);
    }
}

void MotorDM_Disable(Motor_s *motor)
{
    hcan_t *hcan;

    if ((motor == 0) || (motor->initialized == 0U) ||
        (motor->type != MOTOR_TYPE_DM4310))
    {
        return;
    }

    hcan = MotorDM_GetCan(motor->can_bus);
    if (hcan != 0)
    {
        disable_motor_mode(hcan, motor->id, motor->driver.dm4310.mode);
    }
}

void MotorDM_SendCommand(Motor_s *motor)
{
    hcan_t *hcan;

    if ((motor == 0) || (motor->initialized == 0U) ||
        (motor->type != MOTOR_TYPE_DM4310))
    {
        return;
    }

    hcan = MotorDM_GetCan(motor->can_bus);
    if (hcan == 0)
    {
        return;
    }

    switch (motor->control_mode)
    {
        case MOTOR_MODE_MIT:
            mit_ctrl(hcan,
                     motor->id,
                     motor->command.position,
                     motor->command.velocity,
                     motor->command.kp,
                     motor->command.kd,
                     motor->command.torque);
            break;

        case MOTOR_MODE_POSITION:
            pos_speed_ctrl(hcan,
                           motor->id,
                           motor->command.position,
                           motor->command.velocity);
            break;

        case MOTOR_MODE_SPEED:
            speed_ctrl(hcan, motor->id, motor->command.velocity);
            break;

        case MOTOR_MODE_TORQUE:
            mit_ctrl(hcan,
                     motor->id,
                     0.0f,
                     0.0f,
                     0.0f,
                     0.0f,
                     motor->command.torque);
            break;

        case MOTOR_MODE_DISABLED:
        case MOTOR_MODE_CURRENT:
        default:
            break;
    }
}
