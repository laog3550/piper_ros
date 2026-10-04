# 单侧两 CAN 摇操会话的状态快照，由 /teleop/state 以约 2 Hz 发布。
#
# 每一侧各发布一条消息，用 side 字段区分。发布只读取会话内部状态：它不会使能、
# 失能、发送运动帧或改变任何运行参数。

# 侧别：left 或 right。
string side

# 状态名，取值与 docs/TWO_CAN_HOST_TELEOP_IMPLEMENTATION.md 第 6 节一致：
#   OFFLINE            反馈窗口不完整或身份未确认
#   IDLE_DISABLED      整侧已失能，等待启动请求
#   ARMING             正在整侧使能并读回验证
#   WAIT_TEACH         等待操作员用实体按钮让主臂进入示教
#   ALIGNING           从臂正在受限对齐主臂
#   ACTIVE             实时摇操中
#   WAIT_TEACH_RELEASE 已停目标流，等待操作员确认实体示教已退出
#   STOPPING           正在整侧失能并读回验证
#   FAULT_LATCHED      故障已锁存，本进程不再自动进入摇操
string state

# 状态的中文说明，便于直接显示。
string state_label

# 故障原因；没有故障时为空字符串。
string fault

# 是否正在向从臂下发关节目标（即 state == ACTIVE）。
bool running

# 实际下发命令频率，Hz。
float64 command_hz

# 主臂、从臂六关节反馈频率，Hz。
float64 master_hz
float64 follower_hz

# 本会话累计收到的 CAN 错误帧，以及最近 1 秒的速率。
int32 can_error_frames
float64 can_error_rate_hz

# 最近一次计算出的六关节最大跟踪误差，度。
float64 tracking_error_deg

# 两侧夹爪开口，米；反馈不可用时为 NaN。
float64 master_gripper_m
float64 follower_gripper_m
