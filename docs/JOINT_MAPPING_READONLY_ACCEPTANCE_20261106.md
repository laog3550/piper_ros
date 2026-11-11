# 左右臂六关节采集映射只读验收（2026-11-06）

## 验收结论

**现有真实 bag、协议定义及既有现场角色记录，足以支持当前左右主从采集话题的 `teleoperation.mapping_verified=true` 声明。** 左侧 J1–J6 在最新真实 Episode 中均有运动；右侧 J2–J6 在右侧验证 bag 中有运动，右 J1 由 2026-11-03 的双侧真实动作 bag 补齐。独立重解码的 **164674 条六关节 CAN 帧**均与保存的 ROS 弧度值一致；能够按原始 CAN ID 和 Header 找回的 **49743 组向量关节对来源**均与其原始字节一致。没有发现关节对交换、字节序错误、度/弧度混用或额外取反。

这里验收的是**协议关节顺序、单位、目标/反馈同号方向、左右主从来源归属及可追溯的接收时间对应**。不将它扩大为关节绝对零位、每根实体轴相对世界坐标的方向标定、机械跟随时延或每条 Episode 的训练资格。向量停顿、过期和 bag 内来源缺失仍应逐帧拒绝，不能由映射通过豁免。

左右夹爪已分别在[左侧报告][1]与[右侧报告][2]通过，本报告沿用这两个结论，不要求重做夹爪，不将其列为阻断。相机外参、训练归一化与技能标注不参与本次六关节映射判定。

本任务仅新增本报告和[专用只读审计工具][3]，未修改运行配置、历史 Episode 或其他任务文件，未发送 CAN 控制帧，未新建分支或 worktree，未提交或推送 Git。采集配置由主对话统一集成：本次观察到录制器 `idle`、无待处理 Episode，`mapping_verified=false`、`gripper_mapping_verified=true`；配置变更还须在实际应用前重新确认空闲，并确认使用该配置的进程确实重载了它。

## 顺序、单位与方向的依据

以下定义在左右两侧完全相同，没有镜像侧额外负号：

| 主端目标 ID | 从端反馈 ID | `raw_data[0:4]` | `raw_data[4:8]` | `joint_index` | 向量位置 |
| --- | --- | --- | --- | ---: | --- |
| `0x155` | `0x2A5` | J1 | J2 | 1 | `position[0:2]` |
| `0x156` | `0x2A6` | J3 | J4 | 3 | `position[2:4]` |
| `0x157` | `0x2A7` | J5 | J6 | 5 | `position[4:6]` |

每个四字节字段为有符号大端 `int32`，协议单位为 `0.001°`，转换公式为 `rad = raw × π / 180000`。它是绝对关节目标/反馈角，不是每帧增量、速度或电机编码器圈数。表中的字节编号从 0 开始。

官方依据为 SDK V2 的 [CAN ID 定义][4]、[目标角单位][5]、[反馈角单位][6]和[解析器][7]；本机安装版本为 `piper_sdk 0.6.2`。实际驱动见 [`piper_direct_can_decode.py`](../src/piper/piper/piper_direct_can_decode.py)，向量顺序见 [`piper_arm_vector.py`](../src/piper/piper/piper_arm_vector.py)。本次工具不调用这些生产解码函数，直接以 `int.from_bytes(..., 'big', signed=True)` 重算，避免仅用同一实现自证。

目标/反馈同号方向由下表的动态对应支持。表中所有有效动态相关和回归斜率均为正；左六关节回归斜率为 `0.957–1.008`，右 J2–J6 为 `0.981–1.016`。右 J1 的历史快速动作未作机械时延补偿，斜率为 `0.807`、相关系数为 `0.85268`，仍支持同号变化；不能据此宣称增益精度已标定。静止右 J1 的相关系数不定义，工具输出 `null`，没有将其虚构为通过。

仓库 URDF 的 J1–J6 均以**各自关节局部坐标系**的 `axis="0 0 1"` 定义旋转；J2–J6 还具有不同的 `origin.rpy`。因此不能把俯视画面的顺/逆时针当成所有关节共同的正方向，也不能把左右对称安装理解成采集数据需要取反。

## 真实动作覆盖与近时对应

证据来源代号及完整摘要见后文。表中峰峰值为各原始流的 `max-min`，单位为 `°`；`r` 和误差使用同组 ID 的最近 Header 配对，每条目标只选一条反馈，容差为 `30 ms`，允许选到目标之后的反馈。不同目标可能复用同一反馈，原始配对数不等于独立动作次数。

| 侧/关节 | 主要来源 | 目标/反馈峰峰值 `°` | 配对数 | `r` | 绝对差中位数 / P95 `°` |
| --- | --- | ---: | ---: | ---: | ---: |
| 左 J1 | A | 38.907 / 38.540 | 1881 | 0.99704 | 0.233 / 2.964 |
| 左 J2 | A | 112.631 / 112.107 | 1881 | 0.99910 | 0.609 / 4.454 |
| 左 J3 | A | 88.728 / 88.003 | 1881 | 0.99834 | 0.473 / 5.392 |
| 左 J4 | A | 25.022 / 23.778 | 1881 | 0.97851 | 0.635 / 3.093 |
| 左 J5 | A | 87.163 / 86.331 | 1881 | 0.99477 | 0.977 / 5.647 |
| 左 J6 | A | 15.820 / 14.989 | 1881 | 0.96943 | 0.524 / 2.300 |
| 右 J1 | E | 17.729 / 16.036 | 200 | 0.85268 | 2.526 / 4.555 |
| 右 J2 | B | 51.969 / 51.640 | 1682 | 0.99609 | 0.025 / 3.771 |
| 右 J3 | B | 41.108 / 40.574 | 1682 | 0.99600 | 0.555 / 3.314 |
| 右 J4 | B | 28.790 / 28.496 | 1682 | 0.99459 | 0.278 / 3.259 |
| 右 J5 | B | 13.766 / 14.109 | 1682 | 0.98917 | 0.954 / 2.128 |
| 右 J6 | B | 61.240 / 60.362 | 1682 | 0.99209 | 0.308 / 7.323 |

A 的三类目标各 `1881` 条，三类左反馈各 `3162` 条；B 的三类目标各 `1682` 条，三类右反馈各 `6276` 条；E 的右三类目标为 `200/200/201` 条。上表所用目标全部在 `30 ms` 内找到同组反馈。B 的右 J1 目标仅变化 `0.001°`、反馈不变，不用它验收 J1 动态方向。

C、D 为历史左侧重复证据，不计作额外独立的机械臂实体：C 左 J1–J6 的 `r=0.98370–0.99846`，D 为 `0.98888–0.99879`。它们继续支持 A 的六关节同号关系。统计值只是描述现有运动下的对应关系，不构成任意速度、负载下的跟踪误差门限或机器人精度证书。

## 左右与主从实体归属

1. [`config/pi05_can_map.json`](../config/pi05_can_map.json) 固定左适配器序列号 `004B00395443570F20393433`、USB 端口 `1-13:1.0` 到 `can_left`；右适配器序列号 `004C003B5443570F20393433`、端口 `1-4:1.0` 到 `can_right`。本次 `piper_pi05_can verify all --require-up` 两侧均通过。这仅验证适配器，不声称 USB 序列号能区分同总线上的两台臂。
2. [官方原生遥操作现场记录][8]记录了隔离配置后的主端 `0xFA`、从端 `0xFC`、零地址偏移，以及操作员分别移动左右主端并确认对应从端跟随。它建立主目标 `0x155–0x157` 与从反馈 `0x2A5–0x2A7` 的角色背景；当前节点没有从 CAN 帧自动识别机器人序列号的能力。
3. 已通过的左右夹爪现场验收将同一侧总线的目标/反馈与可见实体绑定。六关节帧沿用同一侧、同一角色配置，且本次每条关节帧的 `frame_id`、`joint_index`、每条向量的 `source_topic`、CAN ID 顺序和关节名顺序均符合预期。
4. 本次抽看 A、B、E 的顶部 RGB。A 在约 `0/3/6/9/12 s` 显示左侧实体执行抓取，右侧实体保持原位；B 在约 `0/5/10/15/20/25/30 s` 显示右侧实体运动，左侧保持原位；E 提供两侧分别小幅运动的历史画面及 CAN 动作窗口。这些是抽样视觉佐证，不是逐帧姿态真值。A 的右侧没有任何原始目标帧，右侧目标向量全部 `stale`，不能把这些重复发布的旧目标解释为右侧同时进行了示教。

关键可重查的顶部 RGB Header：

| 来源 | `/camera_top/D455/color/image_raw` 的 Header `unix_ns` | 可见内容 |
| --- | --- | --- |
| A | `1793952329634468994`、`1793952336976685791`、`1793952339980363037` | 初始、左侧抓取、左侧移入盒中 |
| B | `1793950303755546387`、`1793950318740182129`、`1793950328752207031` | 初始、右侧抬起、右侧返回附近 |
| E | `1793690019412330078`、`1793690024414784180`、`1793690039391800537` | 两侧分段小幅移动时段与末尾 |

CAN ID 是消息类型/角色约定，不是发送机器人唯一身份证。以上归属结论依赖既有现场接线、角色配置和夹爪实体绑定持续有效；重接线、角色或偏移变更后应重新核验，不能只复制本报告的布尔值。

## 时间对应通过的范围与不能抹掉的质量信号

原始 Header 是 SocketCAN 帧的主机接收 Unix 时间；bag 的 `messages.timestamp` 是 ROS 接收时间。向量 Header 为四个来源帧时间的最大值，**不是四组关节同时测量，也不是机器人内部采样时间**。本次五个来源中的六关节原始流均无 Header 倒退或同时间冲突；向量 Header 与四来源时间最大值一致，已找到来源的关节值无差异。

近时配对的 `feedback Header - target Header` 中位数：A 三组分别约 `1.256/1.239/1.216 ms`；B 约 `2.419/2.428/2.432 ms`；E 右 J1/J2 约 `0.623 ms`。这些数值只表示两类报文在主机上的接收时间相近。B 的最近反馈可早于目标约 `2.6 ms`，因此不能将最近邻时间差当成机械响应时延，也不能用它证明未来反馈可供在线预测使用。因果训练抽样仍须遵循各自转换器“不晚于采样时刻”的规则。

| 来源/流 | 六关节相关时间问题 | 处理边界 |
| --- | --- | --- |
| A 左目标 | 原始最大间隔 `450.372 ms`；`759` 条向量中 `187` 条含六关节 `stale`，`232` 次 Header 重复；三对关节来源最大跨度 `275.019 ms` | 映射正确不代表这些时刻新鲜。继续使用原始时间与质量标志。 |
| A 右目标 | 原始目标 `0` 条，`792/792` 条向量六关节过期 | 不作为右侧动作证据；不能凭向量发布频率判断主端在发帧。 |
| B 右目标 | 原始最大间隔约 `780.406 ms`；`1569` 条向量中 `1025` 条六关节过期，`1100` 次 Header 重复 | 静止停止发目标与向量重复发布可同时发生；不授予连续训练序列资格。 |
| C、D 左目标 | 原始最大间隔分别约 `212.036/257.451 ms`；D 三对来源最大跨度 `4850.648 ms` | 过期及混合新旧来源仍须拒绝。不能放宽阈值后宣称同步通过。 |
| E 从反馈向量 | 左侧 `81`、右侧 `75` 组关节来源时间落在该 ID 原始流范围内，但 bag 中没有对应原始消息 | 记为未能追溯；不猜测是录制或 DDS 丢失，不以这些来源证明逐帧一致。E 的关节动态比较直接使用 bag 内原始目标和原始反馈。 |

其余未匹配来源由工具分别计为零时间、早于该 ID 的首条原始帧、晚于末条原始帧、原始流根本不存在或原始流内缺失。不能把启动前缓存一概当成数据损坏，也不能补写不存在的原始帧。夹爪历史 `valid/gripper_mapping_verified` 标志不用于否决本次六关节检查；本报告没有改写任何历史标志。

## 可复现的来源与摘要

本地路径均仅作只读访问。C、D 在回收站，本次未恢复、删除或改写；E 在 `/tmp`，需保管原始文件，路径本身不提供长期保存保证。源文件 SHA-256 绑定实际内容；临时审计输出可由命令重建。本次实际输出为 `/tmp/piper_joint_mapping_acceptance_20261106.json`，SHA-256 为 `83815b139dc294eede31b4d0ecd025f90aaeeea444f476658672cb761a16247a`；审计工具 SHA-256 为 `8565ec37bdfcf536a5bda0643fd14e0adf532078a30987839ea50be3c74e0490`。

| 代号 | bag 绝对路径 | 六关节原始帧数 | 找回的向量关节对来源数 |
| --- | --- | ---: | ---: |
| A | `/home/mips/pi05_datas/banana_task_episodes/pick_banana_into_box_left/episode_000004/bag` | 24630 | 6878 |
| B | `/home/mips/pi05_datas/validation/right_gripper_mapping_20261106` | 23874 | 7598 |
| C | `/home/mips/.local/share/Trash/files/episode_000002/bag` | 32723 | 8964 |
| D | `/home/mips/.local/share/Trash/files/episode_000003/bag` | 33282 | 8729 |
| E | `/tmp/piper_full_live_split_probe_20261103/episode_000001/bag` | 50165 | 17574 |

每组来源含两个关节，按向量消息的三对来源逐一计数；同一原始帧被多个向量引用会重复计数，不能将 `49743` 解释为独立 CAN 帧数。

| 代号/文件 | SHA-256 |
| --- | --- |
| A `metadata.yaml` | `a48a7e05141d333656928f566c4eb0354985556ec776ce0f7f65bd1899aee6fb` |
| A `bag_0.db3` | `eb7df4fcaae6e3363aded951928d2d095150d4b9c40fd3e2a936822a1ad97c9c` |
| B `metadata.yaml` | `bdebc79cd74edc27f9f294dfdccdccaff68743f538ce9875e2984b463748d134` |
| B `right_gripper_mapping_20261106_0.db3` | `edf40296509aac6b3cce0b8f1e3194202b82a0b0ce3d0401c2bca107a8c388f0` |
| C `metadata.yaml` | `01a614e009f1df569eaa6a3ff384ac001208443e883dcc4f61aaad6d31f72586` |
| C `bag_0.db3` | `2dc00aa8578e64f4277dc8ad3b5716e5b686827b8f28a2375000ebbb8c513d40` |
| D `metadata.yaml` | `1bbd2e194010d8fc9ccda2c268b97dca3bba997d05848e6021e86b7fb1a8c2f6` |
| D `bag_0.db3` | `ac5a2e0c477b1be5bb2b446954ed2b66c4d0f5d28e2a9b4ca1e06c8d5f4d693d` |
| E `metadata.yaml` | `4c378e5ea4f5279632ff17b8c6286549e64b706e3372fb257048b81925ab2533` |
| E `bag_0.db3` | `3d0c53cbffde7678a6e6bf381bf3d4b5055e072928ab3b2328bd43b11a521785` |

固定接口定义摘要：`PiperDirectCanFrame.msg` 为 `65bce7b992e0e01f2ae4ddbac1e544ad6fd090b83b7120dc6a369fefe6dfdf7c`，`PiperArmVector.msg` 为 `371a8fddc08f38f2c8928c455ca200a3b92ef74e95985684f9433e9d54868ac8`。CAN 映射配置摘要为 `566e54a45e0ee265b02a7162430103de6e60930f5687efdd3b4a35616d7cbd4a`。原始 .db3 已计算全文件摘要，不只依赖 `metadata.yaml`。

复现数值审计（输出文件须尚不存在）：

```bash
cd /home/mips/piper_ros
/tmp/piper-collection-audit-venv/bin/python tools/episode_recorder/audit_joint_mapping.py \
  /home/mips/pi05_datas/banana_task_episodes/pick_banana_into_box_left/episode_000004/bag \
  /home/mips/pi05_datas/validation/right_gripper_mapping_20261106 \
  /home/mips/.local/share/Trash/files/episode_000002/bag \
  /home/mips/.local/share/Trash/files/episode_000003/bag \
  /tmp/piper_full_live_split_probe_20261103/episode_000001/bag \
  --max-pair-ms 30 --output /tmp/piper_joint_mapping_recheck_20261106.json
```

本次环境为 Python 3.10、`rosbags 0.11.7`、`numpy 2.2.6`，现有审计 venv 可用；工具不导入 LeRobot、不需要在线 ROS 节点，不打开 CAN socket、不发布 ROS 话题。它保留每条流的首条原始字节、Header 范围、计数、接收延迟分位数、单位/顺序差异计数、来源可追溯计数及配对分布；不自动翻转任何验收开关。

抽看上述指定 Header 的图像时，可用下列只读方式在本机提取原图；修改 `bag` 和 `wanted` 即可检查其他来源。不使用 `ros2 bag play`，避免向在线订阅者回放历史数据。

```bash
/tmp/piper-collection-audit-venv/bin/python - <<'PY'
from pathlib import Path
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from PIL import Image
import numpy as np
bag = Path('/home/mips/pi05_datas/validation/right_gripper_mapping_20261106')
wanted = {1793950303755546387, 1793950318740182129, 1793950328752207031}
types = get_typestore(Stores.ROS2_HUMBLE)
with Reader(bag) as reader:
    connections = [c for c in reader.connections
                   if c.topic == '/camera_top/D455/color/image_raw']
    for connection, _, raw in reader.messages(connections=connections):
        message = types.deserialize_cdr(raw, connection.msgtype)
        ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if ns not in wanted:
            continue
        assert message.encoding == 'rgb8'
        pixels = np.asarray(message.data, dtype=np.uint8).reshape(message.height, message.step)
        pixels = pixels[:, :message.width * 3].reshape(message.height, message.width, 3)
        output = Path(f'/tmp/joint_mapping_top_{ns}.png')
        with output.open('xb') as stream:
            Image.fromarray(pixels).save(stream, format='PNG')
        wanted.remove(ns)
assert not wanted, wanted
PY
```

已执行独立校验：ROS 原生 CDR 反序列化与本机 SDK 对 A、B、E 的每个存在的侧/角色/关节 ID 各抽查一条，共 `27` 组全部一致；既有直连解码和向量组装测试 `10 passed`；新工具 `py_compile` 与 `git diff --check` 通过。数值审计覆盖全部六关节原始帧，图像检查为上述 Header 抽样，不把二者混为全量视觉验收。

## 如需更强的逐轴现场证据，最小补采内容

本次采集映射声明已有动作证据，不要求为它重新采集全部十二轴。仍未获得的是**有逐轴动作标记、能辨认实体轴的独立正反动作证据**；若后续要将验收升级为逐轴物理方向、精确零位或重新接线后的现场复验，应补这一层，而不是重做夹爪开合：

1. 操作员按既有固件直连流程，在工作区安全的姿态下先左后右；另一侧保持不动。每侧 J1→J6 各做一次缓慢、小幅的去程、静止保持、回程。只要求动作明显超过反馈噪声，不预设未验收的角度、速度或机械零位，也不通过电脑发送目标或角色配置帧。
2. 连续同录双侧八路 `/piper_direct/{left,right}/{master_target,slave_feedback}` 与 `/piper_vector/{left,right}/{master_target,slave_feedback}`、顶部及两腕 RGB。若现有相机看不清实体轴，用能辨认关节编号的现场画面或观察记录补充。事件表至少记 `side`、`joint`、去程/回程、对应原始 Header 范围、操作者观察到的主从实体和运动方向；每段保留静止端点。
3. 使用本工具复核字节、单位与来源；在事件窗口内核对仅预期关节对发生显著变化、两端增减方向一致、另一总线无对应动作。按因果时间顺序另算机械响应时延，不用本报告的最近邻时间差替代。保存原始 bag、事件表和文件 SHA-256。

以上仅为补采步骤，本任务没有启动补采、移动机械臂或触发任何控制流程。

[1]: LEFT_GRIPPER_MAPPING_VALIDATION_20261105.md
[2]: RIGHT_GRIPPER_MAPPING_VALIDATION_20261106.md
[3]: ../tools/episode_recorder/audit_joint_mapping.py
[4]: https://raw.githubusercontent.com/agilexrobotics/piper_sdk/master/piper_sdk/piper_msgs/msg_v2/can_id.py
[5]: https://raw.githubusercontent.com/agilexrobotics/piper_sdk/master/piper_sdk/piper_msgs/msg_v2/transmit/arm_joint_ctrl.py
[6]: https://raw.githubusercontent.com/agilexrobotics/piper_sdk/master/piper_sdk/piper_msgs/msg_v2/feedback/arm_feedback_joint_states.py
[7]: https://raw.githubusercontent.com/agilexrobotics/piper_sdk/master/piper_sdk/protocol/protocol_v2/piper_protocol_v2.py
[8]: OFFICIAL_NATIVE_TELEOP.md
