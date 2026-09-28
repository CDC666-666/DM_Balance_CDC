#include "arm_algorithm.h"

#include "arm_limit.h"

void ArmAlgorithm_Init(ArmAlgorithm_t *algorithm, const float initial_position[ARM_JOINT_COUNT])
{
    unsigned int joint;

    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        algorithm->joint_target[joint] = initial_position[joint];
        algorithm->joint_output[joint] = initial_position[joint];
    }
}

void ArmAlgorithm_SetTarget(ArmAlgorithm_t *algorithm, const float target[ARM_JOINT_COUNT])
{
    unsigned int joint;

    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        algorithm->joint_target[joint] = target[joint];
    }
}

void ArmAlgorithm_Update(ArmAlgorithm_t *algorithm,
                         const float min_position[ARM_JOINT_COUNT],
                         const float max_position[ARM_JOINT_COUNT],
                         const float max_velocity[ARM_JOINT_COUNT],
                         float dt)
{
    unsigned int joint;
    float limited_target;

    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        limited_target = ArmLimit_Clamp(algorithm->joint_target[joint],
                                        min_position[joint],
                                        max_position[joint]);
        algorithm->joint_output[joint] = ArmLimit_Slew(algorithm->joint_output[joint],
                                                        limited_target,
                                                        max_velocity[joint],
                                                        dt);
    }
}
