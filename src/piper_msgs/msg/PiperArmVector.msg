# 四帧组成的七维关节向量；valid 要求帧完整、时间一致、数值有限且夹爪映射已核对。
# header.stamp 为四帧中最新的原始 CAN 时间戳；无帧时为零。frame_id 为 CAN 接口名。
std_msgs/Header header
# 原始输入话题，区分主臂示教目标与从臂实测反馈。
string source_topic
string[7] joint_names
# joint1～joint6 为 rad；gripper 暂按原始计数乘 1e-6 得到 m，方向和开口定义待现场确认。
float64[7] position
# 顺序为关节 1/2、3/4、5/6、夹爪。缺帧时对应时间戳为零、位置填零且 valid=false。
uint32[4] source_can_ids
builtin_interfaces/Time[4] source_stamps
bool[4] missing
bool[4] stale
bool time_inconsistent
bool valid
# 仅现场核对夹爪换算、零位和符号后才能置 true；默认 false，使 valid 保持 false。
bool gripper_mapping_verified
