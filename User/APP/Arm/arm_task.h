#ifndef ARM_TASK_H
#define ARM_TASK_H

#include "arm_types.h"

extern Arm_s arm;

/* 初始化电机管理器和具体机械臂型号。 */
void ArmTask_Init(void);
/* 按固定流水线执行一次机械臂控制更新。 */
void ArmTask_Update(void);
/* FreeRTOS 机械臂任务入口，按配置周期持续调用控制更新。 */
void ArmTask_Run(void const *argument);
/* 通过正式控制链整体更新三关节位置、速度和力矩命令。 */
void Arm_SetCommand(const ArmCommand *command);
/* 更新指定关节的位置目标，不绕过后续安全限制。 */
void Arm_SetJointTarget(uint8_t joint, float position);
/* 更新指定关节的速度目标，不绕过后续安全限制。 */
void Arm_SetJointVelocity(uint8_t joint, float velocity);
/* 锁存人工停止状态并关闭所有已开放关节的输出。 */
void Arm_Disable(void);
/* 返回当前机械臂公共状态机状态。 */
ArmState_e Arm_GetState(void);

#endif /* 机械臂任务接口 */
