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

static Motor_s motor_pool[MOTOR_MANAGER_MAX_MOTORS];
static MotorLookupEntry_s motor_lookup[MOTOR_LOOKUP_CAPACITY];
static uint8_t motor_used_count;

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
    memset(motor_pool, 0, sizeof(motor_pool));
    memset(motor_lookup, 0, sizeof(motor_lookup));
    motor_used_count = 0U;
}

Motor_s *MotorManager_Register(MotorType_e type,
                               uint8_t can_bus,
                               uint8_t id,
                               MotorControlMode_e control_mode)
{
    MotorProtocol_e protocol = Motor_GetProtocol(type);
    MotorLookupEntry_s *entry;
    Motor_s *motor;

    if ((protocol == MOTOR_PROTOCOL_NONE) ||
        (motor_used_count >= MOTOR_MANAGER_MAX_MOTORS) ||
        (MotorManager_FindEntry(motor_lookup, can_bus, protocol, id) != 0))
    {
        return 0;
    }

    entry = MotorManager_FindFreeEntry(motor_lookup,
                                       can_bus,
                                       protocol,
                                       id);
    if (entry == 0)
    {
        return 0;
    }

    motor = &motor_pool[motor_used_count];
    if (MotorInit(motor,
                  id,
                  can_bus,
                  type,
                  control_mode) != MOTOR_STATUS_OK)
    {
        return 0;
    }

    MotorManager_SetEntry(entry, motor, protocol, id);
    motor_used_count++;
    return motor;
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

uint8_t MotorManager_GetRegisteredCount(void)
{
    return motor_used_count;
}
