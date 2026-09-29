#ifndef ARM_MODEL_H
#define ARM_MODEL_H

#include "arm_types.h"

void Arm_ModelInit(Arm_s *arm);
void Arm_ModelUpdateState(Arm_s *arm);
void Arm_ModelHandleException(Arm_s *arm);
void Arm_ModelSetMode(Arm_s *arm);
void Arm_ModelSetReference(Arm_s *arm);
void Arm_ModelCalculate(Arm_s *arm);
void Arm_ModelSafety(Arm_s *arm);
void Arm_ModelSendCommand(Arm_s *arm);

#endif /* ARM_MODEL_H */
