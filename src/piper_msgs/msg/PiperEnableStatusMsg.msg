# 单台 Piper 机械臂六关节驱动器使能状态的只读观测。
#
# 数据来源为各关节驱动器的低速反馈帧（CAN 0x261..0x266，状态字 bit 6）。
# 生成该消息只读取反馈：它不会使能、失能或移动机械臂。

uint8 STATE_UNKNOWN = 0   # 至少有一个关节尚未上报
uint8 STATE_DISABLED = 1  # 六个关节均已上报，且全部未使能
uint8 STATE_PARTIAL = 2   # 六个关节均已上报，部分使能、部分未使能
uint8 STATE_ENABLED = 3   # 六个关节均已上报，且全部已使能

# 关节 1..6 的 driver_enable_status。
bool joint_1_enabled
bool joint_2_enabled
bool joint_3_enabled
bool joint_4_enabled
bool joint_5_enabled
bool joint_6_enabled

# 关节 1..6 是否至少上报过一帧低速反馈。
# 在此之前驱动器报「未使能」仅仅是因为什么都没收到，因此在
# joint_N_valid 为 false 时，joint_N_enabled 必须被忽略。
bool joint_1_valid
bool joint_2_valid
bool joint_3_valid
bool joint_4_valid
bool joint_5_valid
bool joint_6_valid

# 仅当六个关节全部有效且全部使能时为 true。
# 任何部分使能、未知或无效的观测都为 false。
bool all_enabled
# 至少有一个有效关节报使能时为 true。
bool any_enabled
# 聚合后的 STATE_* 取值。
uint8 state

# 本次观测读取自哪个 CAN 接口，用于排除左右串扰。
string can_port
