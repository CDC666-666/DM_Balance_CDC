#include "motor.h"

#include <string.h>

static uint16_t Motor_GetDmDriverMode(MotorControlMode_e control_mode)
{
    switch (control_mode)
    {
        case MOTOR_MODE_POSITION:
            return POS_MODE;

        case MOTOR_MODE_SPEED:
            return SPEED_MODE;

        case MOTOR_MODE_DISABLED:
        case MOTOR_MODE_MIT:
        case MOTOR_MODE_TORQUE:
        default:
            return MIT_MODE;
    }
}

MotorProtocol_e Motor_GetProtocol(MotorType_e type)
{
    switch (type)
    {
        case MOTOR_TYPE_DM4310:
            return MOTOR_PROTOCOL_DM;

        case MOTOR_TYPE_DJI_M3508:
        case MOTOR_TYPE_DJI_M2006:
        case MOTOR_TYPE_DJI_GM6020:
            return MOTOR_PROTOCOL_DJI;

        case MOTOR_TYPE_NONE:
        default:
            return MOTOR_PROTOCOL_NONE;
    }
}

MotorStatus_e MotorInit(Motor_s *motor,
                        uint8_t id,
                        uint8_t can_bus,
                        MotorType_e type,
                        MotorControlMode_e control_mode)
{
    if ((motor == 0) ||
        (id == 0U) ||
        ((can_bus != 1U) && (can_bus != 2U)) ||
        (Motor_GetProtocol(type) == MOTOR_PROTOCOL_NONE))
    {
        return MOTOR_STATUS_INVALID_ARGUMENT;
    }

    if ((type == MOTOR_TYPE_DM4310) &&
        (control_mode == MOTOR_MODE_CURRENT))
    {
        return MOTOR_STATUS_UNSUPPORTED;
    }

    memset(motor, 0, sizeof(*motor));
    motor->id = id;
    motor->type = type;
    motor->control_mode = control_mode;
    motor->can_bus = can_bus;

    if (type == MOTOR_TYPE_DM4310)
    {
        joint_motor_init(&motor->driver.dm4310,
                         motor->id,
                         Motor_GetDmDriverMode(control_mode));
    }

    motor->initialized = 1U;
    return MOTOR_STATUS_OK;
}
