#ifndef ARM_ALGORITHM_H
#define ARM_ALGORITHM_H

#define ARM_JOINT_COUNT 3U

typedef struct
{
    float joint_target[ARM_JOINT_COUNT];
    float joint_output[ARM_JOINT_COUNT];
} ArmAlgorithm_t;

void ArmAlgorithm_Init(ArmAlgorithm_t *algorithm, const float initial_position[ARM_JOINT_COUNT]);
void ArmAlgorithm_SetTarget(ArmAlgorithm_t *algorithm, const float target[ARM_JOINT_COUNT]);
void ArmAlgorithm_Update(ArmAlgorithm_t *algorithm,
                         const float min_position[ARM_JOINT_COUNT],
                         const float max_position[ARM_JOINT_COUNT],
                         const float max_velocity[ARM_JOINT_COUNT],
                         float dt);

#endif /* ARM_ALGORITHM_H */
