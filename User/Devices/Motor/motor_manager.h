#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <stdint.h>

#include "motor.h"

#define MOTOR_MANAGER_MAX_MOTORS 12U

typedef struct
{
    volatile uint32_t feedback_count;
    volatile uint32_t ignored_feedback_count;
    volatile uint8_t registered_count;
    volatile uint8_t last_can_bus;
    volatile uint8_t last_id;
    volatile uint8_t last_motor_type;
} MotorManagerDebug_s;

extern MotorManagerDebug_s motor_manager_debug;

void MotorManager_Init(void);

/* 仅供 MotorInit 内部登记已经完成初始化的电机对象。 */
uint8_t MotorManager_Attach(Motor_s *motor);

Motor_s *MotorManager_GetById(uint8_t can_bus,
                             MotorProtocol_e protocol,
                             uint8_t id);

uint8_t MotorManager_DecodeFeedback(uint8_t can_bus,
                                    MotorProtocol_e protocol,
                                    uint8_t id,
                                    uint8_t *data,
                                    uint32_t data_len);

uint8_t MotorManager_GetRegisteredCount(void);

#endif /* MOTOR_MANAGER_H */
