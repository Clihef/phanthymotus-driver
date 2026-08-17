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

| Action | 关节 | 参数 | 单次默认/范围 |
|---|---|---|---|
| `adjust_ankle` | `ankle_joint` | `ankle_delta_rad` | 默认 0，范围 [-0.03, 0.03] rad |
| `adjust_knee` | `knee_joint` | `knee_delta_rad` | 默认 0，范围 [-0.03, 0.03] rad |
| `adjust_hip` | `hip_joint` | `hip_delta_rad` | 默认 0，范围 [-0.03, 0.03] rad |
| `adjust_waist_yaw` | `waist_yaw_joint` | `waist_yaw_delta_rad` | 默认 0，范围 [-0.03, 0.03] rad |
| `cancel` | 当前运动关节 | 无 | 取消插补并保持最新实测位置 |
| `info` | 全部 | 无 | 返回动作、反馈、控制锁和安全条件 |

四个动作都使用相对实测当前位置的小增量，计算出的绝对目标还会再次通过
`resource/q5_model.urdf` 的关节硬限位。动作要求：

1. `q5_control_mode` 已完成 `prepare_position_control`；
2. `/xbot_state` 新鲜且为 `ACTIVE`；
3. `/joint_states` 新鲜并包含目标关节；
4. `/wr1_controller/commands` 没有其他发布者或卡片持有租约；
5. 完成后收到更新的关节反馈，误差在配置容差内。

承重关节会直接影响机器人稳定性，因此默认配置为
`hardware_enable: false`。实机启用前必须清空工作区、固定或支撑机器人、准备急停，
并从最小增量开始验证。配置中的增量、步长和反馈容差是部署保护值，不是厂商认证限位。

## 本地检查

```bash
cd robotera/q5_bundle
python3 -m unittest test_lower_body_control.py test_q5_bus_bridge.py

# 在安装了 ROS 2 Humble、rclpy 和 Q5 消息包的环境中再运行：
python3 -m unittest test_basic_sensors.py
```

无 ROS 2 环境时至少可以执行语法编译和不依赖硬件的 schema/校验测试；真机动作验证
不属于本地测试，也不能仅凭 URDF 结果认定安全。
