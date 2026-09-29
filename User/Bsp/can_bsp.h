#ifndef _CAN_BSP_H
#define _CAN_BSP_H


#include "main.h"

typedef FDCAN_HandleTypeDef hcan_t;

extern void FDCAN1_Config(void);
extern void FDCAN2_Config(void);
extern uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);

typedef struct
{
    volatile uint32_t attempt_count;
    volatile uint32_t success_count;
    volatile uint32_t failure_count;
    volatile uint16_t last_id;
    volatile uint8_t last_bus;
    volatile uint8_t last_status;
} CanTxDebugInfo;

extern CanTxDebugInfo can_tx_debug;



#endif
