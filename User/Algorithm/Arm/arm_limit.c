#include "arm_limit.h"

static float ArmLimit_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

float ArmLimit_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

float ArmLimit_Slew(float current, float target, float max_velocity, float dt)
{
    float max_step = ArmLimit_Abs(max_velocity) * dt;
    float error = target - current;

    if (error > max_step)
    {
        return current + max_step;
    }
    if (error < -max_step)
    {
        return current - max_step;
    }
    return target;
}

float ArmLimit_MitPosition(float desired_position,
                           float measured_position,
                           float measured_velocity,
                           float kp,
                           float kd,
                           float max_torque)
{
    float damping_torque = ArmLimit_Abs(kd * measured_velocity);
    float position_torque = ArmLimit_Abs(max_torque) - damping_torque;
    float max_error;

    if ((kp <= 0.0f) || (position_torque <= 0.0f))
    {
        return measured_position;
    }

    max_error = position_torque / kp;
    return ArmLimit_Clamp(desired_position,
                          measured_position - max_error,
                          measured_position + max_error);
}
