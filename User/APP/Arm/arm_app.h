#ifndef ARM_APP_H
#define ARM_APP_H

#include <stdint.h>

#include "arm_algorithm.h"

typedef struct
{
    float q[ARM_JOINT_COUNT];
    float dq[ARM_JOINT_COUNT];
    float torque[ARM_JOINT_COUNT];
} ArmCommand;

typedef struct
{
    uint16_t motor_id;
    uint16_t master_id;
    uint8_t motor_version;
    uint8_t can_bus;
    float direction;
    float zero_offset;
    float min_position;
    float max_position;
    float max_velocity;
    float max_torque;
} ArmJointConfig;

typedef enum
{
    ArmStateDisabled = 0,
    ArmStateWaitingFeedback,
    ArmStateRunning,
    ArmStateFault
} ArmState_e;

typedef struct
{
    volatile uint32_t update_count;
    volatile uint32_t enable_request_count;
    volatile uint32_t feedback_count;
    volatile uint16_t last_feedback_id;
    volatile uint8_t last_motor_id;
    volatile uint8_t last_motor_state;
    volatile uint8_t app_state;
} ArmDebugInfo;

extern ArmDebugInfo arm_debug;

void Arm_Init(void);
void Arm_Update(void);
void Arm_SetJointTarget(uint8_t joint, float position);
void Arm_SetCommand(const ArmCommand *command);
void Arm_Stop(void);
void Arm_Disable(void);
ArmState_e Arm_GetState(void);

/* Called only by the CAN BSP after application selection. */
void Arm_OnCanFeedback(uint8_t can_bus,
                       uint16_t master_id,
                       uint8_t *rx_data,
                       uint32_t data_len);

#endif /* ARM_APP_H */
