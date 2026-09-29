#ifndef ARM_TASK_H
#define ARM_TASK_H

#include "arm_types.h"

extern Arm_s arm;

void ArmTask_Init(void);
void ArmTask_Update(void);
void ArmTask_Run(void const *argument);

#endif /* ARM_TASK_H */
