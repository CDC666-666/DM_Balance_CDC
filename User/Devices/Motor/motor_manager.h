#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <stdint.h>

#include "motor.h"

#define MOTOR_MANAGER_MAX_MOTORS 12U

void MotorManager_Init(void);

Motor_s *MotorManager_Register(MotorType_e type,
                               uint8_t can_bus,
                               uint8_t id,
                               MotorControlMode_e control_mode);

Motor_s *MotorManager_GetById(uint8_t can_bus,
                             MotorProtocol_e protocol,
                             uint8_t id);

uint8_t MotorManager_GetRegisteredCount(void);

#endif /* MOTOR_MANAGER_H */
