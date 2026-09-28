#ifndef ARM_LIMIT_H
#define ARM_LIMIT_H

float ArmLimit_Clamp(float value, float minimum, float maximum);
float ArmLimit_Slew(float current, float target, float max_velocity, float dt);
float ArmLimit_MitPosition(float desired_position,
                           float measured_position,
                           float measured_velocity,
                           float kp,
                           float kd,
                           float max_torque);

#endif /* ARM_LIMIT_H */
