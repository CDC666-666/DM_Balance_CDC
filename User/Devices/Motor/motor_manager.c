#include "motor_manager.h"

#include <string.h>

#define MOTOR_LOOKUP_CAPACITY 32U
#define MOTOR_LOOKUP_MASK     (MOTOR_LOOKUP_CAPACITY - 1U)

typedef struct
{
    uint8_t used;
    uint8_t can_bus;
    MotorProtocol_e protocol;
    uint8_t id;
    Motor_s *motor;
} MotorLookupEntry_s;

static MotorLookupEntry_s motor_lookup[MOTOR_LOOKUP_CAPACITY];
static uint8_t motor_used_count;
MotorManagerDebug_s motor_manager_debug;

static uint8_t MotorManager_Hash(uint8_t can_bus,
                                 MotorProtocol_e protocol,
                                 uint8_t id)
{
    uint32_t key = ((uint32_t)can_bus << 24) ^
                   ((uint32_t)protocol << 16) ^
                   (uint32_t)id;
    key ^= key >> 16;
    key *= 0x45D9F3BU;
    key ^= key >> 16;
    return (uint8_t)(key & MOTOR_LOOKUP_MASK);
}

static MotorLookupEntry_s *MotorManager_FindEntry(MotorLookupEntry_s *table,
                                                   uint8_t can_bus,
                                                   MotorProtocol_e protocol,
                                                   uint8_t id)
{
    uint8_t start = MotorManager_Hash(can_bus, protocol, id);
    uint8_t probe;

    for (probe = 0U; probe < MOTOR_LOOKUP_CAPACITY; probe++)
    {
        MotorLookupEntry_s *entry = &table[(start + probe) & MOTOR_LOOKUP_MASK];

        if (entry->used == 0U)
        {
            return 0;
        }
        if ((entry->can_bus == can_bus) &&
            (entry->protocol == protocol) &&
            (entry->id == id))
        {
            return entry;
        }
    }
    return 0;
}

static MotorLookupEntry_s *MotorManager_FindFreeEntry(MotorLookupEntry_s *table,
                                                       uint8_t can_bus,
                                                       MotorProtocol_e protocol,
                                                       uint8_t id)
{
    uint8_t start = MotorManager_Hash(can_bus, protocol, id);
    uint8_t probe;

    for (probe = 0U; probe < MOTOR_LOOKUP_CAPACITY; probe++)
    {
        MotorLookupEntry_s *entry = &table[(start + probe) & MOTOR_LOOKUP_MASK];
        if (entry->used == 0U)
        {
            return entry;
        }
    }
    return 0;
}

static void MotorManager_SetEntry(MotorLookupEntry_s *entry,
                                  Motor_s *motor,
                                  MotorProtocol_e protocol,
                                  uint8_t id)
{
    entry->can_bus = motor->can_bus;
    entry->protocol = protocol;
    entry->id = id;
    entry->motor = motor;
    entry->used = 1U;
}

void MotorManager_Init(void)
{
    memset(motor_lookup, 0, sizeof(motor_lookup));
    memset(&motor_manager_debug, 0, sizeof(motor_manager_debug));
    motor_used_count = 0U;
}

uint8_t MotorManager_Attach(Motor_s *motor)
{
    MotorProtocol_e protocol;
    MotorLookupEntry_s *entry;

    if ((motor == 0) || (motor->initialized == 0U))
    {
        return 0U;
    }

    protocol = Motor_GetProtocol(motor->type);
    if ((protocol == MOTOR_PROTOCOL_NONE) ||
        (motor_used_count >= MOTOR_MANAGER_MAX_MOTORS) ||
        (MotorManager_FindEntry(motor_lookup,
                                motor->can_bus,
                                protocol,
                                motor->id) != 0))
    {
        return 0U;
    }

    entry = MotorManager_FindFreeEntry(motor_lookup,
                                       motor->can_bus,
                                       protocol,
                                       motor->id);
    if (entry == 0)
    {
        return 0U;
    }

    MotorManager_SetEntry(entry, motor, protocol, motor->id);
    motor_used_count++;
    motor_manager_debug.registered_count = motor_used_count;
    return 1U;
}

Motor_s *MotorManager_GetById(uint8_t can_bus,
                             MotorProtocol_e protocol,
                             uint8_t id)
{
    MotorLookupEntry_s *entry = MotorManager_FindEntry(motor_lookup,
                                                        can_bus,
                                                        protocol,
                                                        id);
    return (entry != 0) ? entry->motor : 0;
}

uint8_t MotorManager_DecodeFeedback(uint8_t can_bus,
                                    MotorProtocol_e protocol,
                                    uint8_t id,
                                    uint8_t *data,
                                    uint32_t data_len)
{
    Motor_s *motor = MotorManager_GetById(can_bus, protocol, id);

    if (motor == 0)
    {
        motor_manager_debug.ignored_feedback_count++;
        return 0U;
    }

    Motor_DecodeFeedback(motor, data, data_len);
    motor_manager_debug.feedback_count++;
    motor_manager_debug.last_can_bus = can_bus;
    motor_manager_debug.last_id = id;
    motor_manager_debug.last_motor_type = (uint8_t)motor->type;
    return 1U;
}

uint8_t MotorManager_GetRegisteredCount(void)
{
    return motor_used_count;
}
