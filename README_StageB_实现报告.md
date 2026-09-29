# DM_Balance_CDC Stage B 最终实现与交接报告

## 1. 当前状态

本工程已完成 **Stage B：Motor 与机械臂基础架构搭建**。

- 当前工程：`C:\Users\Lenovo\Desktop\DM_balance\DM`
- 参考工程：`D:\Robomaster\Motor_control\auto_aim\StandardRobotpp`
- 编译器：Keil ARMCC 5.06 update 6 (build 750)
- 默认构建：`ROBOT_TYPE = ROBOT_ARM`
- 验证状态：ARM 与 WheelLeg 均通过完整重建
- Git 状态：本阶段改动仍在工作区，尚未提交

Stage B 说明取代了早期 Stage A 中“不引入通用 Motor 框架”等设计意见。随后根据用户确认，又将 Stage B 最初要求的 command/feedback 双 ID 接口调整为参考工程采用的单逻辑 ID 接口。当前原则是：建立轻量级公共 `Motor_s` 与固定 Arm Task 流水线，但不迁移已经实机验证的 Joint2 旧控制链路。

## 2. 本阶段边界

已完成：

- 通用 `Motor_s`、电机类型和控制模式；
- 静态 Motor Manager、电机池和单逻辑 ID 查找表；
- DM4310 轻量适配层；
- `Arm_s`、`ArmJoint_s`、`ArmFeedback`；
- 固定 Arm Task 流水线和少量 weak model hooks；
- 当前三轴机械臂 model 骨架；
- Joint0、Joint1 禁用配置；
- Keil 工程接入和双机器人模式编译验证。

明确未做：

- 未迁移 Joint2 到新 Motor Manager；
- 未修改 Joint2 CAN 接收、使能、反馈或 MIT 输出路径；
- 未把新 `ArmTask_Run()` 接入 `freertos.c`；
- 未启用 Joint0、Joint1；
- 未实现 DJI Driver、ROS2、IK/FK 或轨迹规划；
- 未修改 WheelLeg 业务逻辑。

三个新 Arm 实现文件仅在 `ROBOT_TYPE == ROBOT_ARM` 时产生实现代码；公共 Motor 层仍在两种构建中保持可编译，以便后续供 Chassis 复用。

## 3. 架构

未来目标数据链：

```text
CAN / Driver
     ↓
Motor Manager
     ↓
   Motor_s
     ↓
Arm / Chassis / Gimbal
```

当前 Stage B 中，新链路只完成静态对象、接口和 DM 适配，不接管运行时 CAN 或电机输出。现有 Joint2 仍使用：

```text
FreeRTOS Arm_Task
→ Arm_Init / Arm_Update
→ arm_joint2_motor
→ mit_ctrl / enable_motor_mode / disable_motor_mode
→ canx_send_data
```

## 4. 文件变更

新增源码：

```text
User/Devices/Motor/motor.h
User/Devices/Motor/motor.c
User/Devices/Motor/motor_manager.h
User/Devices/Motor/motor_manager.c
User/Devices/Motor/motor_dm.h
User/Devices/Motor/motor_dm.c
User/APP/Arm/arm_types.h
User/APP/Arm/arm_config.h
User/APP/Arm/arm_config.c
User/APP/Arm/arm_task.h
User/APP/Arm/arm_task.c
User/APP/Arm/Model/arm_model.h
User/APP/Arm/Model/arm_3dof.c
```

修改：

```text
MDK-ARM/CtrlBoard-H7_IMU.uvprojx
README_机械臂重构架构方案.md（增加历史文档提示）
```

工程文件仅增加 include path 和新源文件。现有 `arm_app`、DM4310 Driver、CAN BSP、FreeRTOS、算法层、`robot_config.h` 与 WheelLeg 源码均无差异。

## 5. Motor 层

### 5.1 类型

`MotorType_e`：

```c
MOTOR_TYPE_NONE
MOTOR_TYPE_DM4310
MOTOR_TYPE_DJI_M3508
MOTOR_TYPE_DJI_M2006
MOTOR_TYPE_DJI_GM6020
```

`MotorControlMode_e`：

```c
MOTOR_MODE_DISABLED
MOTOR_MODE_MIT
MOTOR_MODE_POSITION
MOTOR_MODE_SPEED
MOTOR_MODE_TORQUE
MOTOR_MODE_CURRENT
```

DJI 类型可通过公共 `MotorInit()` 建立通用对象，但 Stage B 仍未实现 DJI 发送和反馈 Driver。

### 5.2 Motor_s

`Motor_s` 保存：

- 电机类型、控制模式；
- CAN Bus；
- 单一逻辑电机编号 `id`；
- 初始化、在线、故障和最后反馈时间；
- 通用反馈与命令；
- `union` 内的 DM4310 私有 `Joint_Motor_t`。

初始化接口：

```c
MotorStatus_e MotorInit(Motor_s *motor,
                        uint8_t id,
                        uint8_t can_bus,
                        MotorType_e type,
                        MotorControlMode_e control_mode);
```

### 5.3 Motor Manager

Motor Manager 使用静态内存：

```text
Motor_s motor_pool[12]
motor_lookup[32]
```

查找表以 `CAN Bus + Protocol Family + Logical ID` 为键，使用开放寻址和线性探测处理哈希冲突。最多注册 12 台电机，查找表容量 32，平均查找复杂度为 O(1)，最坏最多探测 32 个槽位。

注册和查询接口：

```c
Motor_s *MotorManager_Register(...);
Motor_s *MotorManager_GetById(...);
```

业务层只填写一个逻辑 ID。DM4310 按用户约定令 Motor ID 与 Master ID 使用同一编号；DJI 后续由协议层根据电机类型和逻辑 ID 推导反馈帧、控制组与组内槽位。CAN1、CAN2 以及 DM、DJI 协议族仍可使用相同数值的逻辑 ID，不会互相冲突。

没有使用 `malloc`、`new`、STL 或动态链表。

### 5.4 DM4310 Adapter

`motor_dm.c/.h` 保留原 Driver，并只做对象适配：

- 公共 `MotorInit()` 在 DM 类型下完成原 `joint_motor_init()` 所需的私有数据初始化；
- `MotorDM_DecodeFeedback()` → `dm4310_fbdata()`；
- `MotorDM_Enable()` / `MotorDM_Disable()` → 原使能与失能接口；
- `MotorDM_SendCommand()` → 原 MIT、位置速度或速度接口。

Adapter 不会自行运行，也未接入现有 CAN ISR。非 8 字节反馈不会被标记为有效。

## 6. Arm 层

### 6.1 数据结构

`ArmJoint_s` 持有 `Motor_s *`，并保存方向、零点、位置/速度/力矩限制、安装状态和输出许可。

`Arm_s` 保存：

```text
joint[3]
state
mode
command
feedback
update_count
```

`ArmCommand` 复用现有 `arm_app.h` 中的三关节 `q/dq/torque` 定义，避免 Stage B 重复类型或改变旧接口。`ArmFeedback` 提供三关节位置、速度、力矩、在线状态和电机状态。

### 6.2 配置

- Joint0：`installed = 0`，`output_enabled = 0`；
- Joint1：`installed = 0`，`output_enabled = 0`；
- Joint2：新架构预留 `FDCAN1 / Logical ID 6 / MIT`，但 `output_enabled = 0`。

因此新 model 骨架不会向任何关节发送命令，也不会猜测 Joint0、Joint1 参数。当前旧 Joint2 路径仍是 Motor ID 6、Master ID 3；切换到新架构之前，必须先按用户的新约定将电机 Master ID 配置为 6，并完成实机确认。

### 6.3 固定任务流水线

`ArmTask_Update()` 的顺序固定为：

```text
Arm_ModelUpdateState
→ Arm_ModelHandleException
→ Arm_ModelSetMode
→ Arm_ModelSetReference
→ Arm_ModelCalculate
→ Arm_ModelSafety
→ Arm_ModelSendCommand
```

公共入口 `ArmTask_Init()`、`ArmTask_Update()`、`ArmTask_Run()` 不是 weak。型号差异仅通过以下 weak hooks 扩展：

```text
Arm_ModelInit
Arm_ModelUpdateState
Arm_ModelHandleException
Arm_ModelSetMode
Arm_ModelSetReference
Arm_ModelCalculate
Arm_ModelSafety
Arm_ModelSendCommand
```

`arm_3dof.c` 在 `ROBOT_TYPE == ROBOT_ARM && ARM_MODEL_TYPE == ARM_MODEL_3DOF` 时提供同名强定义。Stage B 中它只建立对象和注册配置，所有运行钩子为空，`Arm_ModelSendCommand()` 明确不输出。

## 7. 兼容与安全核对

静态检查结果：

- Joint2 宏仍为 `FDCAN1 / 6 / 3 / MIT`；
- `ARM_JOINT2_SPEED_TEST_ENABLE` 仍为 1，当前 commissioning 行为未改；
- `arm_app.c/.h` 无差异；
- `dm4310_drv.c/.h` 无差异；
- `can_bsp.c/.h` 无差异；
- `Core/Src/freertos.c` 无差异，仍调用旧 `Arm_Init()` / `Arm_Update()`；
- `arm_algorithm`、`arm_limit` 无差异；
- WheelLeg 相关源文件无差异；
- Keil 工程 XML 可正常解析。

## 8. 编译结果

完整重建结果：

```text
ROBOT_TYPE = ROBOT_ARM
Program Size: Code=35536 RO-data=1024 RW-data=232 ZI-data=34568
0 Error(s), 0 Warning(s)

ROBOT_TYPE = ROBOT_WHEEL_LEG
Program Size: Code=63312 RO-data=3236 RW-data=836 ZI-data=36868
0 Error(s), 0 Warning(s)
```

验证后已恢复：

```c
#define ROBOT_TYPE ROBOT_ARM
```

编译通过只能证明源码和链接成立，不能替代上板测试。本阶段刻意没有改变现有运行路径，因此没有执行 Joint2 实机回归；等 Stage C 迁移 Joint2 后必须进行实机回归。

## 9. 下一阶段限制

Stage B 到此停止。未经人工确认，不进入 Stage C。

Stage C 才允许将单个 Joint2 从 `arm_joint2_motor` 迁移到 `Motor_s + Motor Manager + ArmJoint_s + 新 Arm Task`。迁移前必须确认达妙 Motor ID 与 Master ID 均已配置为逻辑 ID 6；迁移时仍不得启用 Joint0、Joint1，并必须重新编译两种机器人模式、完成 Joint2 上板回归。

## 10. 下一阶段交接检查表

开始下一阶段前需要明确：

1. 达妙 Joint2 的 Master ID 是否已经由 3 实际配置为 6；
2. 是否只迁移 Joint2，继续保持 Joint0、Joint1 禁止输出；
3. CAN ISR 是直接用同值逻辑 ID 查表，还是先经过品牌协议映射；
4. 新旧路径切换后如何执行使能、反馈、MIT 输出、超时失能和急停回归；
5. 完成后必须再次构建 `ROBOT_ARM` 与 `ROBOT_WHEEL_LEG`，并进行 Joint2 上板验证。

在第 1 项没有完成前，新 Motor Manager 不能接管当前 Joint2 反馈路径，否则 Motor ID 6 与现有 Master ID 3 不匹配。
