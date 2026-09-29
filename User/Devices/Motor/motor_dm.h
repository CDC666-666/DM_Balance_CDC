#ifndef MOTOR_DM_H
#define MOTOR_DM_H

#include "motor.h"

void MotorDM_DecodeFeedback(Motor_s *motor, uint8_t *data, uint32_t data_len);
void MotorDM_Enable(Motor_s *motor);
void MotorDM_Disable(Motor_s *motor);
void MotorDM_SendCommand(Motor_s *motor);

#endif /* MOTOR_DM_H */
