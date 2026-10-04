#!/usr/bin/env python3
"""Read-only Pi05 CAN identity and configuration checks."""

# The arms now share one bus per side, so a bus is identified by its adapter
# (USB serial plus USB port) and lists the master/follower pair hanging on it.
# Verifying a bus therefore proves the adapter, not how many arms answer on it.

from argparse import ArgumentParser
from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import subprocess
from typing import Dict, Iterable, Optional, Tuple


# Each side has one CAN bus carrying that side's master and follower arm.
BUSES = ('left', 'right')
PIPER_BITRATE = 1_000_000


@dataclass(frozen=True)
class CanBus:
    """Expected SocketCAN and USB identity for one bus and the arms on it."""

    bus: str
    interface: str
    usb_port: str
    serial: str
    bitrate: int
    arms: Tuple[str, str]


def default_config_path() -> Path:
    """Resolve the machine-local mapping without embedding it in code."""
    configured = os.environ.get('PIPER_PI05_CAN_CONFIG')
    if configured:
        return Path(configured).expanduser()
    return Path.home() / 'piper_ros' / 'config' / 'pi05_can_map.json'


def load_config(path: Path) -> Dict[str, CanBus]:
    """Load and strictly validate both Pi05 bus mappings."""
    data = json.loads(path.read_text(encoding='utf-8'))
    if set(data) != set(BUSES):
        raise ValueError('config must contain exactly the two Pi05 buses')
    result = {}
    interfaces = set()
    serials = set()
    for bus in BUSES:
        values = data[bus]
        arms = values.get('arms')
        if not isinstance(arms, list):
            raise ValueError(f'invalid arms list for {bus}')
        entry = CanBus(
            bus=bus,
            interface=str(values['interface']),
            usb_port=str(values['usb_port']),
            serial=str(values['serial']),
            bitrate=int(values['bitrate']),
            arms=tuple(str(arm) for arm in arms),
        )
        if not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}', entry.interface):
            raise ValueError(f'invalid interface for {bus}')
        if not re.fullmatch(r'[A-Za-z0-9_.-]+', entry.serial):
            raise ValueError(f'invalid USB serial for {bus}')
        if not re.fullmatch(r'[0-9-]+:[0-9.]+', entry.usb_port):
            raise ValueError(f'invalid USB port for {bus}')
        if entry.bitrate != PIPER_BITRATE:
            raise ValueError(f'{bus} bitrate must be {PIPER_BITRATE}')
        expected_arms = {f'master_{bus}', f'follower_{bus}'}
        if len(entry.arms) != 2 or set(entry.arms) != expected_arms:
            raise ValueError(
                f'{bus} bus must carry {sorted(expected_arms)}, '
                f'got {sorted(entry.arms)}'
            )
        if entry.interface in interfaces:
            raise ValueError(f'duplicate interface: {entry.interface}')
        if entry.serial in serials:
            raise ValueError(f'duplicate USB serial for {bus}')
        interfaces.add(entry.interface)
        serials.add(entry.serial)
        result[bus] = entry
    return result


def _serial_for(interface: str, sys_class_net: Path) -> str:
    device = sys_class_net / interface / 'device'
    try:
        resolved = device.resolve(strict=True)
    except (FileNotFoundError, OSError):
        return ''
    for candidate in (resolved, *resolved.parents):
        try:
            serial = (candidate / 'serial').read_text().strip()
        except (FileNotFoundError, OSError):
            continue
        if serial:
            return serial
    return ''


def _link_details(interface: str) -> str:
    completed = subprocess.run(
        ['ip', '-details', 'link', 'show', 'dev', interface],
        check=False,
        capture_output=True,
        text=True,
    )
    return completed.stdout if completed.returncode == 0 else ''


def _usb_port(interface: str) -> str:
    completed = subprocess.run(
        ['ethtool', '-i', interface],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        return ''
    match = re.search(r'^bus-info:\s*(\S+)', completed.stdout, re.MULTILINE)
    return match.group(1) if match else ''


def _bitrate(details: str) -> Optional[int]:
    match = re.search(r'\bbitrate\s+(\d+)\b', details)
    return int(match.group(1)) if match else None


def _is_up(details: str) -> bool:
    first_line = details.splitlines()[0] if details else ''
    flags = re.search(r'<([^>]*)>', first_line)
    return bool(flags and 'UP' in flags.group(1).split(','))


def verify_bus(
    entry: CanBus,
    require_up: bool = False,
    sys_class_net: Path = Path('/sys/class/net'),
) -> Iterable[str]:
    """Return all mismatches without changing interface state."""
    # Checks the adapter that defines the bus, not which or how many arms hang
    # on it: piper_bus_probe and piper_joint_watch answer that part.
    problems = []
    if not (sys_class_net / entry.interface).exists():
        return [f'{entry.interface}: interface missing']
    actual_serial = _serial_for(entry.interface, sys_class_net)
    if actual_serial != entry.serial:
        problems.append(
            f'{entry.interface}: USB serial mismatch for the {entry.bus} bus'
        )
    actual_usb_port = _usb_port(entry.interface)
    if actual_usb_port != entry.usb_port:
        problems.append(
            f'{entry.interface}: USB port is {actual_usb_port or "missing"}, '
            f'expected {entry.usb_port} for the {entry.bus} bus'
        )
    details = _link_details(entry.interface)
    actual_bitrate = _bitrate(details)
    if actual_bitrate != entry.bitrate:
        problems.append(
            f'{entry.interface}: bitrate is {actual_bitrate}, '
            f'expected {entry.bitrate}'
        )
    if require_up and not _is_up(details):
        problems.append(f'{entry.interface}: link is DOWN')
    return problems


def _can_interfaces() -> Iterable[str]:
    completed = subprocess.run(
        ['ip', '-brief', 'link', 'show', 'type', 'can'],
        check=True,
        capture_output=True,
        text=True,
    )
    return [line.split()[0] for line in completed.stdout.splitlines()]


def _run_ip(*arguments: str) -> None:
    subprocess.run(['ip', *arguments], check=True)


def activate(mapping: Dict[str, CanBus]) -> None:
    """Apply the confirmed mapping and bring both links up at 1 Mbps."""
    if os.geteuid() != 0:
        raise PermissionError('activate must be run with sudo')
    current_interfaces = tuple(_can_interfaces())
    by_bus = {}
    for bus, entry in mapping.items():
        matches = [
            interface for interface in current_interfaces
            if _serial_for(interface, Path('/sys/class/net')) == entry.serial
            and _usb_port(interface) == entry.usb_port
        ]
        if len(matches) != 1:
            raise RuntimeError(
                f'{bus}: expected one adapter matching serial and USB port, '
                f'found {len(matches)}'
            )
        by_bus[bus] = matches[0]
    if len(set(by_bus.values())) != len(BUSES):
        raise RuntimeError('one CAN adapter matched more than one bus')

    temporary = {
        bus: f'p5tmp{index}' for index, bus in enumerate(BUSES)
    }
    occupied = set(current_interfaces)
    collisions = occupied.intersection(temporary.values())
    if collisions:
        raise RuntimeError(f'temporary interface exists: {sorted(collisions)}')

    for bus in BUSES:
        interface = by_bus[bus]
        _run_ip('link', 'set', 'dev', interface, 'down')
        _run_ip('link', 'set', 'dev', interface, 'name', temporary[bus])
    for bus in BUSES:
        entry = mapping[bus]
        interface = temporary[bus]
        _run_ip('link', 'set', 'dev', interface, 'name', entry.interface)
        _run_ip('link', 'set', 'dev', entry.interface, 'down')
        _run_ip(
            'link', 'set', 'dev', entry.interface,
            'type', 'can', 'bitrate', str(entry.bitrate),
        )
        _run_ip('link', 'set', 'dev', entry.interface, 'up')


def _masked(serial: str) -> str:
    """Show both ends: adapters of one batch share their trailing digits."""
    if len(serial) <= 10:
        return '...'
    return f'{serial[:4]}...{serial[-6:]}'


def _parser() -> ArgumentParser:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument(
        '--config', type=Path, default=default_config_path(),
        help='machine-local Pi05 JSON mapping',
    )
    subparsers = parser.add_subparsers(dest='command', required=True)
    subparsers.add_parser('show', help='show the configured mapping')
    verify = subparsers.add_parser('verify', help='verify one or both buses')
    verify.add_argument('bus', choices=(*BUSES, 'all'))
    verify.add_argument(
        '--require-up', action='store_true',
        help='also fail if a matching interface is DOWN',
    )
    activate_parser = subparsers.add_parser(
        'activate', help='rename, configure, and bring up both CAN links'
    )
    activate_parser.add_argument(
        '--apply', action='store_true',
        help='required acknowledgement for changing network link state',
    )
    return parser


def main(args=None) -> int:
    """Run the read-only Pi05 CAN configuration utility."""
    options = _parser().parse_args(args)
    try:
        mapping = load_config(options.config)
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f'ERROR: cannot load {options.config}: {exc}')
        return 2
    if options.command == 'show':
        print(
            f'{"BUS":6} {"INTERFACE":10} {"USB-PORT":14} {"BITRATE":>7} '
            f'{"ADAPTER":14} ARMS'
        )
        for bus in BUSES:
            entry = mapping[bus]
            print(
                f'{bus:6} {entry.interface:10} {entry.usb_port:14} '
                f'{entry.bitrate:7} {_masked(entry.serial):14} '
                f'{" + ".join(entry.arms)}'
            )
        return 0
    if options.command == 'activate':
        if not options.apply:
            print('ERROR: activate requires --apply')
            return 2
        try:
            activate(mapping)
        except (OSError, PermissionError, RuntimeError,
                subprocess.CalledProcessError) as exc:
            print(f'ERROR: activation failed: {exc}')
            return 1
        print('Pi05 CAN activation completed')
        return 0
    buses = BUSES if options.bus == 'all' else (options.bus,)
    failed = False
    for bus in buses:
        entry = mapping[bus]
        problems = list(verify_bus(entry, options.require_up))
        if problems:
            failed = True
            for problem in problems:
                print(f'[FAIL] {problem}')
        else:
            state = 'UP required' if options.require_up else 'identity matched'
            arms = ' + '.join(entry.arms)
            print(f'[OK]   {bus}: {entry.interface} ({arms}), {state}')
    return int(failed)


if __name__ == '__main__':
    raise SystemExit(main())
