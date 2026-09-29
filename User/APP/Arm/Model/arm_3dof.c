#include "arm_model.h"

#include <string.h>

#include "arm_config.h"
#include "motor_manager.h"
#include "robot_config.h"

#if (ROBOT_TYPE == ROBOT_ARM) && (ARM_MODEL_TYPE == ARM_MODEL_3DOF)

void Arm_ModelInit(Arm_s *arm_instance)
{
    uint8_t joint;

    if (arm_instance == 0)
    {
        return;
    }

    memset(arm_instance, 0, sizeof(*arm_instance));
    arm_instance->state = ArmStateDisabled;
    arm_instance->mode = ArmModeDisabled;

    for (joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        const ArmJointConfig_s *config = &arm_3dof_joint_config[joint];
        ArmJoint_s *arm_joint = &arm_instance->joint[joint];

        arm_joint->direction = config->direction;
        arm_joint->zero_offset = config->zero_offset;
        arm_joint->min_position = config->min_position;
        arm_joint->max_position = config->max_position;
        arm_joint->max_velocity = config->max_velocity;
        arm_joint->max_torque = config->max_torque;
        arm_joint->installed = config->installed;
        arm_joint->output_enabled = config->output_enabled;

        if (config->installed != 0U)
        {
            arm_joint->motor = MotorManager_Register(config->motor_type,
                                                      config->can_bus,
                                                      config->id,
                                                      config->control_mode);
        }
    }
}

void Arm_ModelUpdateState(Arm_s *arm_instance)
{
    (void)arm_instance;
}

void Arm_ModelHandleException(Arm_s *arm_instance)
{
    (void)arm_instance;
}

void Arm_ModelSetMode(Arm_s *arm_instance)
{
    (void)arm_instance;
}

void Arm_ModelSetReference(Arm_s *arm_instance)
{
    (void)arm_instance;
}

void Arm_ModelCalculate(Arm_s *arm_instance)
{
    (void)arm_instance;
}

void Arm_ModelSafety(Arm_s *arm_instance)
{
    (void)arm_instance;
}

void Arm_ModelSendCommand(Arm_s *arm_instance)
{
    /* Stage B deliberately produces no motor output. */
    (void)arm_instance;
}

#endif /* ROBOT_TYPE == ROBOT_ARM && ARM_MODEL_TYPE == ARM_MODEL_3DOF */
