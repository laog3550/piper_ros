#!/usr/bin/env python3
"""Configure an isolated master for two-CAN host teleoperation."""

from argparse import ArgumentParser
import math

import can

from piper.piper_bus_probe import survey
from piper.piper_feedback import HIGH_SPEED_CAN_IDS, commanding_can_ids

LINKAGE_CAN_ID = 0x470
OUTPUT_ARM = 0xFC
EXIT_OK = 0
EXIT_REFUSED = 1
EXIT_FAILED = 3


def build_payload() -> bytes:
    """Encode only the supported host-teleop master configuration."""
    return bytes((OUTPUT_ARM, 0x20, 0x20, 0, 0, 0, 0, 0))


def send_frame(port: str, payload: bytes) -> None:
    """Send one maintenance frame; never enable or move an arm."""
    bus = can.Bus(interface='socketcan', channel=port,
                  receive_own_messages=False)
    try:
        bus.send(can.Message(arbitration_id=LINKAGE_CAN_ID,
                             data=payload, is_extended_id=False))
    finally:
        bus.shutdown()


def single_arm(counter, seconds: float) -> bool:
    """Check both shared driver IDs and every possible pose window."""
    driver_rates = [counter.get(i, 0) / seconds for i in HIGH_SPEED_CAN_IDS]
    windows = [[counter.get(i + offset, 0) / seconds
                for i in range(0x2A1, 0x2A9)] for offset in (0, 0x10, 0x20)]
    active = [rates for rates in windows if any(rates)]
    return (all(150 <= rate < 300 for rate in driver_rates)
            and len(active) == 1
            and all(150 <= rate < 300 for rate in active[0]))


def _parser() -> ArgumentParser:
    parser = ArgumentParser(description='离线配置主臂为 0xFC + 0x20 偏移')
    parser.add_argument('--port', required=True, help='仅配置这一条 CAN 接口')
    parser.add_argument('--output-arm', action='store_true',
                        help='兼容方案命令；始终使用 0xFC，不支持固件随动')
    parser.add_argument('--feedback-offset', choices=('0x20',), default='0x20')
    parser.add_argument('--ctrl-offset', choices=('0x20',), default='0x20')
    parser.add_argument('--linkage-offset', choices=('0x00',), default='0x00')
    parser.add_argument('--survey-seconds', type=float, default=2.0)
    parser.add_argument('--isolated-master', action='store_true',
                        help='确认仅主臂上电连接，从臂已断电或断开 CAN')
    parser.add_argument('--send', action='store_true', help='发送一次配置；默认只读')
    return parser


def main(args=None) -> int:
    """Require physical isolation and one observed feedback source."""
    options = _parser().parse_args(args)
    seconds = options.survey_seconds
    if not math.isfinite(seconds) or seconds <= 0:
        print('拒绝：观察时间必须为有限正数')
        return EXIT_REFUSED
    if options.send and not options.isolated_master:
        print('拒绝：0x470 是广播帧；从臂断电或断开 CAN 后，'
              '加 --isolated-master 确认只连接主臂')
        return EXIT_REFUSED
    payload = build_payload()
    print(f'{options.port}：0x470 {payload.hex(" ")}（离线维护，不用于退出）')
    try:
        counts, _, _ = survey(options.port, seconds)
        if not single_arm(counts, seconds) or commanding_can_ids(counts):
            print('拒绝：必须仅有一套单臂反馈，且没有控制或配置流')
            return EXIT_REFUSED
        if not options.send:
            print('干跑完成：没有发送任何帧')
            return EXIT_OK
        send_frame(options.port, payload)
        print('已发送一次配置；请主臂断电重启后只读核对 0x2C1～0x2C8。'
              '发送成功不代表配置已读回或已持久保存。')
        return EXIT_OK
    except (OSError, can.CanError) as exc:
        print(f'失败：{exc}')
        return EXIT_FAILED


if __name__ == '__main__':
    raise SystemExit(main())
