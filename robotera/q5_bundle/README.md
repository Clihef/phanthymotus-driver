# RobotEra Q5 Bundle

RobotEra Q5 的 MCP 驱动 bundle。插件由 `main.py` 按 `config.yaml` 动态加载，
状态来自 Q5 ROS 2 Domain 211，画布数据通过 bridge 转发给 Agent Core。

## 控制卡安全约定

- 所有 `dispatch()` 返回普通 Python 字典，由 `main.py` 统一包装 MCP content；
  `ok: false`、`state: error` 或包含 `error` 的结果会由入口标记为 MCP `isError`。
- 物理动作必须检查新鲜反馈、Q5 FSM、控制模式、参数范围和共享发布器租约。
- `arm_control`、`head_control`、`arm_gesture` 和 `lower_body_control` 共用
  `/wr1_controller/commands`，不得并发发布互相冲突的身体指令。
- UI 参数必须在 JSON Schema 中声明范围、默认值、说明和 `x-action-params`。

## 下半身控制卡

`lower_body_control` 将四个下半身关节合并为一张卡：

| Action | 关节 | 参数 | 默认值 / Q5 URDF 范围 |
|---|---|---|---|
| `set_ankle` | `ankle_joint` | `ankle_position_deg` | 0° / [0°, 92.24°] |
| `set_knee` | `knee_joint` | `knee_position_deg` | 0° / [-144.95°, 29.79°] |
| `set_hip` | `hip_joint` | `hip_position_deg` | 0° / [-29.79°, 89.95°] |
| `set_waist_yaw` | `waist_yaw_joint` | `waist_yaw_position_deg` | 0° / [-89.95°, 89.95°] |
| `cancel` | 当前运动关节 | 无 | 取消插补并保持最新实测位置 |
| `info` | 全部 | 无 | 返回动作、反馈、控制锁和安全条件 |

四个动作都接收角度制的绝对关节位置，内部转换为弧度后发送。目标会再次通过
`resource/q5_model.urdf` 的关节硬限位。动作执行时会自动完成以下前置流程：

1. 下半身采用独立的最小直控准备流程 `pos → READY → ACTIVE`，不会执行手臂专用的
   `initpose_handsdown` 或 `lift_up`；仅处于 `ACTIVE` 不能证明当前已由位置直控接管；
2. 模式切换后必须同时收到新鲜 `ACTIVE` 状态和一帧切换后的新 `/joint_states`，才把
   下半身直控标记为已准备并允许发命令；
3. `/joint_states` 新鲜并包含目标关节；
4. `/wr1_controller/commands` 上保留但静默的遥控器/MPC端点可以共存；检测到其实际
   命令流、监测状态未确定、其他活跃发布者、重复的 `q5_body_command` 或其他卡片持有
   租约时拒绝执行，运动过程中恢复外部发布也会中止本卡片命令；
5. 完成后收到更新的关节反馈，误差在配置容差内；超时时返回最后一帧反馈时间，便于
   区分“反馈停止”与“反馈仍在更新但关节没有跟随”。

承重关节会直接影响机器人稳定性。当前配置启用硬件执行；测试时必须清空工作区、
固定或支撑机器人并准备急停。插补步长、速度和反馈容差是部署保护值，不是厂商认证限位。

## 本地检查

```bash
cd robotera/q5_bundle
python3 -m unittest test_lower_body_control.py test_q5_bus_bridge.py

# 在安装了 ROS 2 Humble、rclpy 和 Q5 消息包的环境中再运行：
python3 -m unittest test_basic_sensors.py
```

无 ROS 2 环境时至少可以执行语法编译和不依赖硬件的 schema/校验测试；真机动作验证
不属于本地测试，也不能仅凭 URDF 结果认定安全。
