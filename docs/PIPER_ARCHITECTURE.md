# Piper 控制架构

## 目标

控制链路遵循“单一命令所有者、会话管理生命周期、算法与 ROS 适配分离”三条规则。
同一台 follower 的命令话题在任意时刻只能有一个发布者；使能、遥操作、回位与可选失能
属于同一个遥操作会话，不再由互不感知的命令行进程拼接。

## 模块边界

| 模块 | 职责 | 扩展方式 |
| --- | --- | --- |
| `piper_interfaces.py` | 集中定义左右臂话题、服务和默认 master 话题 | 新命名空间先在此增加兼容映射 |
| `piper_teleop_node.py` | ROS 订阅、发布、使能服务和命令所有权检查 | 新传输层实现相同的桥接接口 |
| `piper_teleop_cli.py` | 命令行契约、默认值和参数校验 | 新参数只在此登记和校验 |
| `piper_teleop.py` | 对齐、跟随、快速复位、回位状态机 | 新控制策略实现独立策略对象后注入 |
| `piper_feedback.py` | 反馈解码与纯算法工具 | 按“解码、轨迹、滤波”继续拆分 |

## 会话生命周期

```text
检查反馈与命令所有权
        ↓
可选请求使能（--manage-enable）
        ↓
对齐 → 跟随 → 可选快速复位
        ↓
回到启动姿态（除非显式关闭或被中断）
        ↓
可选退出失能（--disable-on-exit）
```

`start_fast_two_teleop.launch.py` 为左右侧各启动一个会话节点。每个节点只拥有本侧命令
话题，并在发布运动指令前检查是否已有其他发布者。旧的 `ros2 service call` 子进程已移除，
避免服务调用退出并不代表机械臂已进入预期状态的问题。

## 命名约定

Python 文件、函数和变量使用 `snake_case`，类使用 `PascalCase`，常量使用
`UPPER_SNAKE_CASE`。新增 ROS 接口应按 `/<domain>/<side>/<role>/<resource>` 组织；现有
`/joint_ctrl_cmd_left` 等接口为了现场兼容暂不直接改名，统一从 `piper_interfaces.py`
取得，后续可以在一个位置增加新旧命名迁移层。

## 后续拆分建议

`piper_feedback.py` 仍混合 CAN 反馈、限位、轨迹规划和四种滤波器，应在后续 PR 中拆成
`feedback_decode.py`、`motion_profile.py` 与 `filters.py`。该拆分应保持纯函数 API，并先
迁移现有测试，避免与现场控制逻辑重构同时进行。
