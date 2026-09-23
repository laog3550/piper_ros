#!/usr/bin/env python3
"""Read-only Pi05 CAN identity and configuration checks."""

from argparse import ArgumentParser
from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import subprocess
from typing import Dict, Iterable, Optional


ROLES = (
    'master_left',
    'master_right',
    'follower_left',
    'follower_right',
)
PIPER_BITRATE = 1_000_000


@dataclass(frozen=True)
class CanRole:
    """Expected SocketCAN and USB identity for one Piper arm."""

    role: str
    interface: str
    usb_port: str
    serial: str
    bitrate: int


def default_config_path() -> Path:
    """Resolve the machine-local mapping without embedding it in code."""
    configured = os.environ.get('PIPER_PI05_CAN_CONFIG')
    if configured:
        return Path(configured).expanduser()
    return Path.home() / 'piper_ros' / 'config' / 'pi05_can_map.json'


def load_config(path: Path) -> Dict[str, CanRole]:
    """Load and strictly validate all four Pi05 role mappings."""
    data = json.loads(path.read_text(encoding='utf-8'))
    if set(data) != set(ROLES):
        raise ValueError('config must contain exactly the four Pi05 roles')
    result = {}
    interfaces = set()
    serials = set()
    for role in ROLES:
        values = data[role]
        entry = CanRole(
            role=role,
            interface=str(values['interface']),
            usb_port=str(values['usb_port']),
            serial=str(values['serial']),
            bitrate=int(values['bitrate']),
        )
        if not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}', entry.interface):
            raise ValueError(f'invalid interface for {role}')
        if not re.fullmatch(r'[A-Za-z0-9_.-]+', entry.serial):
            raise ValueError(f'invalid USB serial for {role}')
        if not re.fullmatch(r'[0-9-]+:[0-9.]+', entry.usb_port):
            raise ValueError(f'invalid USB port for {role}')
        if entry.bitrate != PIPER_BITRATE:
            raise ValueError(f'{role} bitrate must be {PIPER_BITRATE}')
        if entry.interface in interfaces:
            raise ValueError(f'duplicate interface: {entry.interface}')
        if entry.serial in serials:
            raise ValueError(f'duplicate USB serial for {role}')
        interfaces.add(entry.interface)
        serials.add(entry.serial)
        result[role] = entry
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


def verify_role(
    entry: CanRole,
    require_up: bool = False,
    sys_class_net: Path = Path('/sys/class/net'),
) -> Iterable[str]:
    """Return all mismatches without changing interface state."""
    problems = []
    if not (sys_class_net / entry.interface).exists():
        return [f'{entry.interface}: interface missing']
    actual_serial = _serial_for(entry.interface, sys_class_net)
    if actual_serial != entry.serial:
        problems.append(
            f'{entry.interface}: USB serial mismatch for {entry.role}'
        )
    actual_usb_port = _usb_port(entry.interface)
    if actual_usb_port != entry.usb_port:
        problems.append(
            f'{entry.interface}: USB port is {actual_usb_port or "missing"}, '
            f'expected {entry.usb_port} for {entry.role}'
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


def activate(mapping: Dict[str, CanRole]) -> None:
    """Apply the confirmed mapping and bring all four links up at 1 Mbps."""
    if os.geteuid() != 0:
        raise PermissionError('activate must be run with sudo')
    current_interfaces = tuple(_can_interfaces())
    by_role = {}
    for role, entry in mapping.items():
        matches = [
            interface for interface in current_interfaces
            if _serial_for(interface, Path('/sys/class/net')) == entry.serial
            and _usb_port(interface) == entry.usb_port
        ]
        if len(matches) != 1:
            raise RuntimeError(
                f'{role}: expected one adapter matching serial and USB port, '
                f'found {len(matches)}'
            )
        by_role[role] = matches[0]
    if len(set(by_role.values())) != len(ROLES):
        raise RuntimeError('one CAN adapter matched more than one role')

    temporary = {
        role: f'p5tmp{index}' for index, role in enumerate(ROLES)
    }
    occupied = set(current_interfaces)
    collisions = occupied.intersection(temporary.values())
    if collisions:
        raise RuntimeError(f'temporary interface exists: {sorted(collisions)}')

    for role in ROLES:
        interface = by_role[role]
        _run_ip('link', 'set', 'dev', interface, 'down')
        _run_ip('link', 'set', 'dev', interface, 'name', temporary[role])
    for role in ROLES:
        entry = mapping[role]
        interface = temporary[role]
        _run_ip('link', 'set', 'dev', interface, 'name', entry.interface)
        _run_ip('link', 'set', 'dev', entry.interface, 'down')
        _run_ip(
            'link', 'set', 'dev', entry.interface,
            'type', 'can', 'bitrate', str(entry.bitrate),
        )
        _run_ip('link', 'set', 'dev', entry.interface, 'up')


def _masked(serial: str) -> str:
    return f'...{serial[-6:]}' if len(serial) > 6 else '...'


def _parser() -> ArgumentParser:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument(
        '--config', type=Path, default=default_config_path(),
        help='machine-local Pi05 JSON mapping',
    )
    subparsers = parser.add_subparsers(dest='command', required=True)
    subparsers.add_parser('show', help='show the configured mapping')
    verify = subparsers.add_parser('verify', help='verify one or all roles')
    verify.add_argument('role', choices=(*ROLES, 'all'))
    verify.add_argument(
        '--require-up', action='store_true',
        help='also fail if a matching interface is DOWN',
    )
    activate_parser = subparsers.add_parser(
        'activate', help='rename, configure, and bring up all four CAN links'
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
        for role in ROLES:
            entry = mapping[role]
            print(
                f'{role:16} {entry.interface:8} {entry.usb_port:14} '
                f'{entry.bitrate:7} '
                f'{_masked(entry.serial)}'
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
    roles = ROLES if options.role == 'all' else (options.role,)
    failed = False
    for role in roles:
        entry = mapping[role]
        problems = list(verify_role(entry, options.require_up))
        if problems:
            failed = True
            for problem in problems:
                print(f'[FAIL] {problem}')
        else:
            state = 'UP required' if options.require_up else 'identity matched'
            print(f'[OK]   {role}: {entry.interface}, {state}')
    return int(failed)


if __name__ == '__main__':
    raise SystemExit(main())
