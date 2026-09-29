# DM_Balance_CDC 机械臂重构架构方案

> **历史文档：** 本文记录 Stage A 分析，其中部分设计已被后续 Stage B 任务说明推翻。当前实现、验收结果和后续边界请以 [`README_StageB_实现报告.md`](README_StageB_实现报告.md) 为准。

## 1. 文档状态

- 当前阶段：Stage A（架构分析）
- 当前工程：`C:\Users\Lenovo\Desktop\DM_balance\DM`
- 参考工程：`D:\Robomaster\Motor_control\auto_aim\StandardRobotpp`
- 参考工程来源：`https://gitee.com/cai-decheng/fa-pancake.git`
- 参考提交：`8cda45758d3a636b5f20fec2f75d3e059d2906f4`
- 当前结论：方案待确认，尚未修改机械臂源码

本次重构的首要原则是：

> 结构可以变，但已经验证工作的 Joint2 电机链路不能因为重构而改变。

---

## 2. 已确认的事实与前提修正

### 2.1 Joint2 当前配置

当前实际生效配置为：

```text
CAN Bus  = FDCAN1
Motor ID = 6
Master ID = 3
Mode = MIT
```

`User/APP/robot_config.h` 中仍有一条旧注释写着“command ID 8、feedback/master ID 4”，但实际宏定义和控制代码使用的是 `6 / 3`。该注释已过期，后续应修正注释，不能修改实际参数。

### 2.2 当前实机验证范围

当前配置中：

```c
#define ARM_JOINT2_SPEED_TEST_ENABLE 1
```

因此目前直接得到验证的是连续速度调试路径：

```text
使能
→ CAN 反馈
→ 状态判断
→ MIT 速度命令
→ 反馈超时保护
```

正常的：

```text
ArmCommand
→ 位置限位
→ Slew Rate
→ MIT 位置控制
```

在当前构建中被条件编译掉。不能仅凭 Joint2 已经转动，就断言所有正式位置控制功能均完成了实机验证。

### 2.3 当前项目已经具备的基础

当前代码并非完全从单轴裸代码开始，已经具备：

- `ArmCommand`；
- 三元素 `ArmJointConfig` 数组；
- `arm_algorithm` 和 `arm_limit`；
- `ROBOT_ARM / ROBOT_WHEEL_LEG` 编译期隔离；
- Arm Task 与 CAN BSP 的基本接线。

主要问题是运行时对象、状态、安全和输出逻辑仍然硬编码为 Joint2。

---

## 3. 已阅读文件

### 3.1 当前工程

```text
User/APP/robot_config.h
User/APP/Arm/arm_app.c
User/APP/Arm/arm_app.h
User/Algorithm/Arm/arm_algorithm.c
User/Algorithm/Arm/arm_algorithm.h
User/Algorithm/Arm/arm_limit.c
User/Algorithm/Arm/arm_limit.h
User/Devices/DM_Motor/dm4310_drv.c
User/Devices/DM_Motor/dm4310_drv.h
User/Bsp/can_bsp.c
User/Bsp/can_bsp.h
Core/Src/freertos.c
Core/Src/main.c
MDK-ARM/CtrlBoard-H7_IMU.uvprojx
```

### 3.2 参考工程

```text
application/mechanical_arm/mechanical_arm.c/.h
application/mechanical_arm/mechanical_arm_task.c/.h
application/mechanical_arm/mechanical_arm_engineer.c/.h
application/mechanical_arm/mechanical_arm_penguin_mini.c/.h
application/robot_cmd/motor.c/.h
application/robot_cmd/CAN_receive.c/.h
application/robot_cmd/CAN_cmd_damiao.c/.h
application/robot_param_engineer.h
application/robot_param_penguin_mini_arm.h
Src/freertos.c
```

参考工程工作区存在其他未提交改动，但机械臂、Motor 和达妙命令文件未显示为修改。分析以这些未修改模块为主；已修改的 `freertos.c` 仅用于理解任务接线。

---

## 4. 当前 DM_Balance_CDC 架构

### 4.1 调用链

```text
FreeRTOS Arm Task
→ Arm_Init / Arm_Update
→ Arm 状态机、限位、测试逻辑
→ mit_ctrl
→ canx_send_data
→ FDCAN1
→ DM4310 Joint2
```

反馈链：

```text
DM4310 Joint2
→ FDCAN1
→ HAL_FDCAN_RxFifo0Callback
→ Arm_OnCanFeedback
→ dm4310_fbdata
→ arm_joint2_motor
```

### 4.2 结构性问题

1. 存在独立的 `arm_joint2_motor`，没有统一运行时关节数组。
2. 反馈时间和反馈有效标志是机械臂全局量，不能区分三个关节。
3. CAN 选择、坐标转换、使能、失能和 MIT 输出均硬编码到 Joint2。
4. 关节参数同时存在于 `robot_config.h`、配置数组和限位数组中。
5. `ARM_JOINT_COUNT` 定义在算法头文件中，App 数据类型反向依赖 Algorithm。
6. Commissioning 与正式控制通过大段条件编译互斥。
7. `arm_app.c` 同时承担状态机、硬件绑定、安全限制、调试和输出。
8. 当前电机故障分支有时只进入 `Fault`，没有立即发送 disable。这是现有安全行为问题，不应在纯结构迁移中无说明地改变。
9. `Arm_SetCommand()` 暂无调用者；未来由通信任务调用时，需要约定单写者或临界区策略。

---

## 5. fa-pancake 实际架构

参考工程的机械臂任务采用固定流水线：

```text
MechanicalArmPublish
→ MechanicalArmInit
→ MechanicalArmObserver
→ MechanicalArmHandleException
→ MechanicalArmSetMode
→ MechanicalArmReference
→ MechanicalArmConsole
→ MechanicalArmSendCmd
```

主要设计：

- 使用单个 `MechanicalArm_s` 保存机械臂状态；
- 使用 `joint_motor[]` 表示全部关节电机；
- 使用通用 `Motor_s` 保存电机信息、反馈值和设定值；
- CAN ISR 按总线、电机类型和 ID 写入公共反馈缓存；
- 控制任务通过 `GetMotorMeasure()` 获取反馈；
- 达妙发送包装函数根据电机对象中的 CAN 和 ID 选择硬件；
- 不同机械臂型号通过编译期开关和弱函数实现。

参考工程没有独立、完整的 Kinematics 或 Trajectory 层。它仍然存在大量逐关节宏，且型号实现文件中混合了遥控器、模式、安全、控制和发送逻辑。

---

## 6. 值得借鉴与不应照搬的设计

### 6.1 值得借鉴

- 单一机械臂对象；
- `joint[]` 数组；
- 固定周期控制流水线；
- 电机反馈和设定值集中到对象；
- ISR 只进行接收、匹配和解析；
- 参数配置与运行状态分离。

### 6.2 不应照搬

- 不引入同时支持 DJI、CyberGear、LK、DM 的大型通用电机框架；
- 不采用弱函数覆盖整套机械臂实现；
- 不采用仅按电机 ID 索引反馈缓存的方案；
- 不复制逐关节配置宏；
- 不复制微秒级阻塞发送延时；
- 不复制参考项目中的遥控器、气泵和五/六轴耦合逻辑；
- 不重写当前已验证的 DM4310 Driver 和 CAN BSP。

---

## 7. 建议目标架构

```text
FreeRTOS Arm Task
        │
        ▼
     Arm App
        │
        ├── Arm Command / Feedback
        ├── Arm State
        ├── Global Safety
        │
        ▼
   Arm Trajectory
        │
        ▼
     Arm Joint
        │
        ├── Joint/Motor 坐标转换
        ├── 单关节限位
        ├── 在线状态
        └── 输出许可
        │
        ▼
  Existing DM4310 Driver
        │
        ▼
  Existing CAN BSP
```

本阶段不创建空的 `arm_kinematics`。有真实 FK、IK 或 Jacobian 后再增加，并保证它不依赖 CAN 或 DM4310 Driver。

---

## 8. 建议目录结构

```text
User/
├── APP/
│   ├── robot_config.h
│   └── Arm/
│       ├── arm_app.c
│       ├── arm_app.h
│       ├── arm_types.h
│       ├── arm_config.c
│       ├── arm_config.h
│       ├── arm_joint.c
│       ├── arm_joint.h
│       ├── arm_commissioning.c
│       └── arm_commissioning.h
│
├── Algorithm/
│   └── Arm/
│       ├── arm_trajectory.c
│       ├── arm_trajectory.h
│       ├── arm_limit.c
│       └── arm_limit.h
│
├── Devices/
│   └── DM_Motor/
│       ├── dm4310_drv.c
│       └── dm4310_drv.h
│
└── Bsp/
    ├── can_bsp.c
    └── can_bsp.h
```

暂不单独创建只有少量包装函数的 `arm_safety`：全局安全状态由 `arm_app` 负责，单关节限制由 `arm_joint + arm_limit` 负责。当安全规则增长后再拆分。

---

## 9. 数据结构设计

### 9.1 ArmJointConfig

```c
typedef struct
{
    uint8_t installed;
    uint8_t output_enabled;
    uint8_t can_bus;
    uint8_t motor_version;

    uint16_t motor_id;
    uint16_t master_id;

    float direction;
    float zero_offset;

    float min_position;
    float max_position;
    float max_velocity;
    float max_torque;

    float kp;
    float kd;
} ArmJointConfig;
```

`installed` 表示关节硬件配置是否有效；`output_enabled` 单独控制是否允许使能和发送命令。Joint0、Joint1 默认二者均为 0。

### 9.2 ArmJoint_t

```c
typedef struct
{
    Joint_Motor_t motor;
    const ArmJointConfig *config;

    volatile uint32_t last_feedback_ms;
    volatile uint8_t feedback_seen;
} ArmJoint_t;
```

职责：

- 保存现有 DM4310 电机实例；
- Joint/Motor 坐标转换；
- 使能、失能和 MIT 输出包装；
- 单关节在线判断；
- 单关节位置、速度和力矩限制。

### 9.3 ArmCommand

```c
typedef struct
{
    float q[ARM_JOINT_COUNT];
    float dq[ARM_JOINT_COUNT];
    float torque[ARM_JOINT_COUNT];
} ArmCommand;
```

### 9.4 ArmFeedback

```c
typedef struct
{
    float q[ARM_JOINT_COUNT];
    float dq[ARM_JOINT_COUNT];
    float torque[ARM_JOINT_COUNT];

    uint32_t last_feedback_ms[ARM_JOINT_COUNT];
    uint8_t online[ARM_JOINT_COUNT];
    uint8_t state[ARM_JOINT_COUNT];
} ArmFeedback;
```

### 9.5 Arm_t

```c
typedef struct
{
    ArmJoint_t joint[ARM_JOINT_COUNT];

    ArmState_e state;
    ArmCommand command;
    ArmFeedback feedback;
    ArmTrajectory_t trajectory;

    uint32_t start_ms;
    uint32_t last_enable_ms;
} Arm_t;
```

整个工程只保留一个机械臂实例，不再增加 `arm_joint0_motor`、`arm_joint1_motor`、`arm_joint2_motor`。

---

## 10. CAN 反馈分发方案

CAN BSP 接口保持不变：

```c
Arm_OnCanFeedback(can_bus, rx_id, rx_data, data_len);
```

内部逻辑：

```text
遍历 arm.joint[0..2]
→ 检查 installed
→ 匹配 can_bus
→ 匹配 master_id 与接收帧 ID
→ 调用原 dm4310_fbdata()
→ 更新时间戳和 feedback_seen
```

ISR 中不进行：

- Joint/Motor 坐标转换；
- 在线状态计算；
- 轨迹控制；
- 运动学；
- 状态机更新；
- 控制输出。

这些工作由 1 kHz Arm Task 完成。

---

## 11. Joint2 行为保持方案

重构过程中必须固定：

```text
FDCAN1
Motor ID = 6
Master ID = 3
MIT_MODE
direction = +1
zero_offset = 0
```

同时保持：

- 原 `dm4310_fbdata()`；
- 原 `mit_ctrl()`；
- 原 enable/disable 命令；
- 原启动重试周期；
- 原反馈超时阈值；
- 原速度、力矩、KP、KD 参数；
- 原状态判断和调用顺序。

结构迁移阶段不顺带修改现有故障处理行为。安全行为增强应作为单独变更进行评审和实机验证。

---

## 12. Joint0 / Joint1 策略

Joint0、Joint1 仅预留数组元素和接口：

```text
installed = 0
output_enabled = 0
```

在真实 CAN ID、Master ID、方向、零点、限位和力矩参数得到确认前：

- 不发送 enable；
- 不发送控制帧；
- 不执行运动；
- 不猜测硬件参数。

---

## 13. Commissioning 策略

当前连续速度测试和 Local Test 将迁移到：

```text
arm_commissioning.c/.h
```

要求：

- 正式构建默认关闭；
- 仅对 `output_enabled` 的关节生效；
- 开启 Joint2 速度测试后，保持当前参数：
  - 目标速度 `π rad/s`；
  - MIT `KD = 0.10`；
  - 最大力矩预算 `0.50 N·m`；
- 调试代码不覆盖正式 `ArmCommand`；
- 调试结束仍可正常 disable。

注意：当前速度测试默认值为 1，而目标方案要求默认关闭。这是明确、可见的默认行为变化，不是无意的重构副作用。

---

## 14. WheelLeg 隔离策略

保持现有编译期隔离：

```text
ROBOT_TYPE == ROBOT_WHEEL_LEG
→ 原任务
→ 原 Chassis 控制
→ 原 CAN switch 分发

ROBOT_TYPE == ROBOT_ARM
→ Arm Task
→ Arm CAN 分发
```

不修改：

- WheelLeg 任务；
- WheelLeg 电机对象；
- WheelLeg CAN ID；
- FDCAN 初始化和波特率；
- `canx_send_data()`。

新增 Arm 文件不得在全局初始化阶段发送 CAN 或改变硬件状态。

---

## 15. ROS2 接入点

未来通信模块只依赖：

```c
void Arm_SetCommand(const ArmCommand *command);
void Arm_GetFeedback(ArmFeedback *feedback);
ArmState_e Arm_GetState(void);
```

建议通信链：

```text
ROS2 / Communication
→ ArmCommand
→ Arm App
→ Arm Joint
→ DM4310
```

反馈链：

```text
DM4310
→ Arm Joint
→ ArmFeedback
→ Communication
→ ROS2 /joint_states
```

通信模块不能直接访问 CAN BSP 或 DM4310 Driver。

---

## 16. 文件变更计划

### 16.1 新增

```text
User/APP/Arm/arm_types.h
User/APP/Arm/arm_config.c
User/APP/Arm/arm_config.h
User/APP/Arm/arm_joint.c
User/APP/Arm/arm_joint.h
User/APP/Arm/arm_commissioning.c
User/APP/Arm/arm_commissioning.h
User/Algorithm/Arm/arm_trajectory.c
User/Algorithm/Arm/arm_trajectory.h
```

### 16.2 修改

```text
User/APP/robot_config.h
User/APP/Arm/arm_app.c
User/APP/Arm/arm_app.h
MDK-ARM/CtrlBoard-H7_IMU.uvprojx
```

### 16.3 迁移后删除

```text
User/Algorithm/Arm/arm_algorithm.c
User/Algorithm/Arm/arm_algorithm.h
```

删除前必须确认 `arm_trajectory` 已完整接管原 Clamp/Slew 行为，并完成两个机器人模式的编译。

### 16.4 保持不动

```text
User/Devices/DM_Motor/dm4310_drv.c/.h
User/Bsp/can_bsp.c/.h
Core/Src/fdcan.c
Core/Src/main.c
Core/Src/freertos.c
WheelLeg 相关源文件
```

---

## 17. 分阶段实施计划

### Stage B：统一数据结构

- 增加 `Arm_t`、`ArmJoint_t`、`ArmJointConfig`、`ArmCommand`、`ArmFeedback`；
- 建立配置表；
- 暂不迁移现有 Joint2 输出路径；
- 编译 ROBOT_ARM 和 ROBOT_WHEEL_LEG。

### Stage C：Joint2 迁移

- 将 `arm_joint2_motor` 迁移到 `arm.joint[2].motor`；
- 保持原参数和调用顺序；
- Joint0/Joint1 不输出；
- 编译两个机器人模式。

### Stage D：CAN Feedback 重构

- 使用 `CAN Bus + Master ID` 遍历匹配三关节；
- ISR 仍只做必要解析；
- 编译两个机器人模式。

### Stage E：Commissioning 拆分

- 迁移连续速度测试和 Local Test；
- 正式默认关闭；
- 开启时保持当前 Joint2 调试行为；
- 编译两个机器人模式。

### Stage F：Joint0 / Joint1 预留

- 完成统一三关节对象；
- 配置保持未安装、禁止输出；
- 不填写猜测参数；
- 编译两个机器人模式。

每个阶段必须在上一个阶段 `0 Error` 后才能继续。不能通过注释代码、删除功能或关闭警告绕过问题。

---

## 18. 验证计划

### 18.1 静态检查

- DM4310 Driver 无差异；
- CAN BSP 无行为差异；
- Joint2 配置仍为 `FDCAN1 / 6 / 3`；
- Joint0/Joint1 输出许可为 0；
- ISR 中没有轨迹或运动学调用；
- 无 `malloc/new`。

### 18.2 编译检查

使用 Keil ARMCC 5.06 分别重建：

```text
ROBOT_TYPE = ROBOT_ARM
ROBOT_TYPE = ROBOT_WHEEL_LEG
```

要求：

```text
0 Error
```

当前已有历史构建日志显示两个模式曾达到 `0 Error / 0 Warning`，但 WheelLeg 日志早于当前提交，不能替代重构后的重新构建。

### 18.3 Joint2 实机回归

编译通过只能证明代码和链接正确，不能证明硬件回归完成。最终需要上板验证：

1. 正确初始化；
2. 正确使能；
3. 收到 Master ID 3 反馈；
4. 正确解析 position、velocity、torque；
5. MIT 速度调试行为一致；
6. 正式命令路径限位正确；
7. 反馈超时后停止输出并 disable；
8. 手动 disable 正常；
9. Joint0/Joint1 无任何控制帧；
10. WheelLeg 固件行为不受影响。

---

## 19. 已知风险

1. 当前验证重点是速度调试路径，正式位置控制仍需要单独上板验证。
2. ISR 与 Arm Task 共享电机反馈结构，结构迁移阶段保持现状；如未来出现反馈一致性问题，再评估快照或双缓冲。
3. `Arm_SetCommand()` 将来可能跨任务调用，需要规定唯一写入者或增加轻量临界区。
4. 参考工程的逐关节宏和通用多品牌电机框架不适合直接复制。
5. 将 commissioning 默认关闭会改变当前固件上电后的默认连续转动行为，但这是任务说明明确要求的安全调整。
6. 结构重构不能替代真实机械限位、急停和断电保护。

---

## 20. 待确认事项

开始 Stage B 前需要确认：

1. 同意采用本文的最小分层方案；
2. 同意不重写 DM4310 Driver 和 CAN BSP；
3. 同意 Joint0/Joint1 保持未安装、禁止输出；
4. 同意最终将连续速度 commissioning 改为默认关闭；
5. 同意每阶段分别编译 ROBOT_ARM 和 ROBOT_WHEEL_LEG 后再继续。
