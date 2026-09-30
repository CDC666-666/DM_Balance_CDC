# Stage C 三关节迁移与 Joint0 单轴验证报告

## 1. 阶段结论

本阶段已经完成三关节机械臂向统一 `Motor_s + Motor Manager + Arm_s + Arm Task` 控制链的迁移。

根据后续明确要求，最终结构进一步调整为参考工程的使用方式：

- 机械臂任务循环、状态处理、参考量生成、控制计算、安全检查和命令发送全部实现在 `arm_task.c`；
- 机械臂直接持有三个 `Motor_s` 对象；
- Arm 初始化电机时只调用 `MotorInit()`；
- `MotorInit()` 完成具体驱动初始化后，内部自动把电机指针登记到 Motor Manager；
- Motor Manager 不再分配电机对象，只维护 `CAN Bus + Protocol + ID` 到 `Motor_s *` 的查找表。

当前默认构建配置已经恢复为：

```c
#define ROBOT_TYPE ROBOT_ARM
```

Joint0 是唯一允许使能和发送控制命令的关节，Joint1、Joint2 只注册和接收反馈。

## 2. 已删除的旧代码

已删除以下旧测试和旧架构内容：

- `ARM_JOINT2_SPEED_TEST_ENABLE`
- `ARM_JOINT2_SPEED_RAD_S`
- `ARM_JOINT2_SPEED_MIT_KD`
- `ARM_LOCAL_TEST_ENABLE`
- `ARM_LOCAL_TEST_STEP_MS`
- `Arm_RunContinuousSpeed()`
- `Arm_UpdateLocalTest()`
- 旧 Joint2 独立速度测试和位置阶跃测试
- 旧 `Arm_Init()`、`Arm_Update()`、`Arm_OnCanFeedback()` 运行链
- `arm_app.c`、`arm_app.h`
- 独立的 `Model/arm_3dof.c`、`Model/arm_model.h`
- Keil 工程中对应的旧源文件和 Model include path

没有重新增加独立 Test Mode。Joint0 的 1 圈/秒命令进入正式的命令、参考量、计算、安全和发送流水线。

## 3. 三关节最终配置

| 关节 | 电机 | 模式 | CAN | ID | installed | output_enabled | 位置范围 | 最大速度 | 软件力矩预算 |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| Joint0 | DM4310 | MIT | FDCAN1 | 1 | 1 | 1 | -4.7124～4.7124 rad | 20.944 rad/s | 3.5 N·m |
| Joint1 | DM4310 | MIT | FDCAN1 | 2 | 1 | 0 | -1.10～1.20 rad | 20.944 rad/s | 3.5 N·m |
| Joint2 | DM4310 | MIT | FDCAN1 | 3 | 1 | 0 | -2.50～0 rad | 20.944 rad/s | 3.5 N·m |

ID 规则为：

```text
Joint0：CAN ID = Master ID = 1
Joint1：CAN ID = Master ID = 2
Joint2：CAN ID = Master ID = 3
```

注意：三个关节统一配置到 FDCAN1 是依据当前工程连接方式作出的配置。此前参考工程中的总线编号不是本项目硬件接线证明，烧录前必须核对实际线束。

## 4. 电机初始化和登记

Arm 侧每个关节只调用一次：

```c
MotorInit(&arm_motor[joint], id, can_bus, type, control_mode);
```

调用链为：

```text
ArmTask_Init
→ MotorInit
→ 初始化 Motor_s 和 DM4310 Driver 数据
→ MotorManager_Attach
→ 建立 CAN Bus + Protocol + ID 查找项
```

`MotorManager_GetRegisteredCount()` 的预期结果为 3，三个 `arm.joint[i].motor` 均指向 `arm_task.c` 中静态持有的真实电机对象。没有使用 `malloc/new`。

## 5. CAN 反馈调用链

ARM 模式的反馈路径为：

```text
FDCAN1/FDCAN2 接收中断
→ MotorManager_DecodeFeedback
→ 按 CAN Bus + DM Protocol + ID 查找 Motor_s
→ Motor_DecodeFeedback
→ MotorDM_DecodeFeedback
→ dm4310_fbdata
→ 更新 feedback、last_feedback_ms、online、fault
```

中断中没有执行机械臂状态机、参考量生成、控制计算、安全判断或命令发送。

保留的调试量包括：

- `motor_manager_debug.feedback_count`
- `motor_manager_debug.ignored_feedback_count`
- `motor_manager_debug.registered_count`
- 最近收到的 CAN、ID 和电机类型
- 每个电机的反馈、在线状态、故障和反馈时间戳

## 6. 公共 Motor API

Arm 不直接依赖 `motor_dm.h`，调用链为：

```text
Arm
→ Motor_Enable / Motor_Disable / Motor_SendCommand
→ 按 motor->type 分发
→ MotorDM_*
→ 原 DM4310 Driver
→ CAN
```

`dm4310_drv.c/.h` 未修改，现有协议编码继续复用。

## 7. Arm Task 单文件控制流水线

FreeRTOS 仅运行：

```text
Arm_Task
→ ArmTask_Run
```

`arm_task.c` 中的每周期固定顺序为：

```text
Arm_UpdateState
→ Arm_HandleException
→ Arm_SetMode
→ Arm_SetReference
→ Arm_Calculate
→ Arm_Safety
→ Arm_SendCommand
```

以上函数及其实现均在同一个 `arm_task.c` 中，没有 weak hook 或另一个型号 `.c` 覆盖运行逻辑。

## 8. 各阶段实现

### UpdateState

统一遍历三个关节，从各自 `Motor_s.feedback` 更新位置、速度、力矩、在线状态和电机状态。统一坐标变换为：

```text
joint_position = direction × (motor_position - zero_offset)
motor_position = direction × joint_position + zero_offset
```

反馈超过 `ARM_FEEDBACK_TIMEOUT_MS` 后将对应电机标记为离线。

### HandleException

人工停止请求优先级最高。对允许输出的关节检查空指针、未初始化、离线、未使能、电机故障和启动超时。

Joint1、Joint2 的 `output_enabled = 0`，因此它们可以继续接收和显示反馈，但不会阻塞 Joint0 的单轴验证。

首次收到有效反馈后，把实际关节位置复制到位置命令，避免上电默认位置 0 引发突然回零。

### SetMode

只有 `ArmStateRunning` 对应 `ArmModeNormal`，其余状态统一为 `ArmModeDisabled`。

### SetReference

把 `arm.command` 中三个关节的位置、速度和力矩统一限幅后写入 `arm.reference`。

### Calculate

三个关节共用一套计算逻辑，将关节坐标命令转换为电机坐标命令，并写入 MIT 的位置、速度、力矩、Kp、Kd 字段。

### Safety

每周期先清除 `output_allowed`，只有满足以下条件才重新放行：

- 关节已安装；
- 配置允许输出；
- Arm 状态为 Running；
- 电机已初始化、在线且无故障；
- 速度和力矩经过限制。

到达机械位置边界后禁止继续向外运动，但允许反向退出。Kd 会根据速度误差动态收紧，使阻尼项不超过 3.5 N·m 的连续力矩预算。

### SendCommand

统一遍历三个关节，只有同时满足 `installed`、`output_enabled` 和 `output_allowed` 才调用 `Motor_SendCommand()`。

## 9. 只有 Joint0 输出的保证

三处共同保证只有 Joint0 发出主动控制：

1. 配置表中只有 Joint0 的 `output_enabled = 1`；
2. 使能函数只遍历并使能 `output_enabled` 关节；
3. Safety 和 SendCommand 都再次检查 `output_enabled`。

因此 Joint1、Joint2 不会发送 Enable、MIT、位置、速度、力矩控制命令，也没有为了测试而注释掉发送代码。

## 10. Joint0 当前速度和力矩参数

用户指定目标速度为：

```text
1 圈/秒 = 60 rpm = 6.28318531 rad/s
```

DM-J4310-2EC V1.2 24 V 手册参数为额定转速 120 rpm、最大空载转速 200 rpm、额定力矩 3.5 N·m、峰值力矩 12.5 N·m。代码采用：

- 200 rpm，即 20.944 rad/s，作为速度硬限制；
- 3.5 N·m 额定力矩作为持续软件预算；
- 不把 12.5 N·m 峰值力矩当作可持续限值。

资料来源：[DM-J4310-2EC V1.2 电机使用手册](https://aifitlab-wiki.super.site/damiao-docs/dm-j4310-2ec-v12-motor-instruction-manual)

重要说明：Joint0 配置了 ±1.5π 的机械位置范围，所以当前 1 圈/秒命令不会无限连续旋转；到达正向限位后，Safety 会把继续向外的速度命令置零。如果实际机构允许连续旋转，需要先重新确认机械结构、线束缠绕风险和位置反馈过圈策略，不能直接删除限位。

## 11. robot_config.h 清理结果

已删除所有旧 Joint2 测试宏和本地阶跃测试宏。当前只保留：

- 机器人类型选择；
- Arm 控制周期；
- 反馈超时；
- 启动超时；
- 使能重试周期。

关节 ID、总线、方向、零点、限位和控制参数均集中在 `arm_config.c/.h`。

## 12. 编译结果

使用 Keil ARMCC V5.06 update 6 完整 Rebuild：

| 构建配置 | 结果 | 程序尺寸 |
|---|---|---|
| ROBOT_ARM | 0 Error，0 Warning | Code 38072，RO 1048，RW 232，ZI 35312 |
| ROBOT_WHEEL_LEG | 0 Error，0 Warning | Code 63312，RO 3236，RW 836，ZI 36868 |

日志：

- `MDK-ARM/build_stagec_arm_final.log`
- `MDK-ARM/build_stagec_wheelleg_final.log`

最终再次构建了 ROBOT_ARM，因此当前 `.axf/.hex` 对应机械臂配置。

## 13. Joint0 上板前检查清单

1. 机械固定可靠，Joint0 运动范围内无人和障碍物；
2. 确认实际电机为 DM4310 V1.2，供电电压与手册版本一致；
3. Joint0 的 CAN ID 和 Master ID 均为 1；
4. Joint1 的 CAN ID 和 Master ID 均为 2；
5. Joint2 的 CAN ID 和 Master ID 均为 3；
6. 三个 ID 不冲突；
7. 确认三个电机实际连接 FDCAN1；若接线不同，先修改配置表；
8. 确认 Joint0 的 `direction = +1` 与实物正方向一致；
9. 确认零点和 ±1.5π 限位适合当前机构；
10. 初次上电准备硬件急停或立即断电手段；
11. Debug Watch 确认 registered count 为 3；
12. 确认三个 `arm.joint[i].motor` 均非空；
13. 确认 Joint0 online、fault、state 和反馈时间戳正常；
14. 确认 CAN TX 只出现 ID 1 的主动使能和控制帧；
15. 确认 ID 2、ID 3 没有主动控制输出；
16. 验证反馈超时、故障和人工 `Arm_Disable()` 都能关闭 Joint0 输出。

## 14. 本阶段停止位置

代码只开放 Joint0，未开放 Joint1、Joint2，也未执行真实硬件测试。下一阶段之前应先完成 Joint0 的 Enable、Feedback、State、Reference、Calculate、Safety、CAN TX 和 Fault/Disable 全链路实机回归。
