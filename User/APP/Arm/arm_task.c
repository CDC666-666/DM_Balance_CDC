#include "arm_task.h"

#include "arm_model.h"
#include "cmsis_os.h"
#include "motor_manager.h"
#include "robot_config.h"

#if ROBOT_TYPE == ROBOT_ARM

Arm_s arm;

__weak void Arm_ModelInit(Arm_s *arm_instance)
{
    (void)arm_instance;
}

__weak void Arm_ModelUpdateState(Arm_s *arm_instance)
{
    (void)arm_instance;
}

__weak void Arm_ModelHandleException(Arm_s *arm_instance)
{
    (void)arm_instance;
}

__weak void Arm_ModelSetMode(Arm_s *arm_instance)
{
    (void)arm_instance;
}

__weak void Arm_ModelSetReference(Arm_s *arm_instance)
{
    (void)arm_instance;
}

__weak void Arm_ModelCalculate(Arm_s *arm_instance)
{
    (void)arm_instance;
}

__weak void Arm_ModelSafety(Arm_s *arm_instance)
{
    (void)arm_instance;
}

__weak void Arm_ModelSendCommand(Arm_s *arm_instance)
{
    (void)arm_instance;
}

void ArmTask_Init(void)
{
    MotorManager_Init();
    Arm_ModelInit(&arm);
}

void ArmTask_Update(void)
{
    Arm_ModelUpdateState(&arm);
    Arm_ModelHandleException(&arm);
    Arm_ModelSetMode(&arm);
    Arm_ModelSetReference(&arm);
    Arm_ModelCalculate(&arm);
    Arm_ModelSafety(&arm);
    Arm_ModelSendCommand(&arm);
    arm.update_count++;
}

void ArmTask_Run(void const *argument)
{
    (void)argument;
    ArmTask_Init();

    for (;;)
    {
        ArmTask_Update();
        osDelay(ARM_CONTROL_PERIOD_MS);
    }
}

#endif /* ROBOT_TYPE == ROBOT_ARM */
