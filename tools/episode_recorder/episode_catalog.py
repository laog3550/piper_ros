"""只读扫描保存目录中的实际 Episode，供记录表刷新使用。"""

import json
from pathlib import Path
import re


EPISODE_DIRECTORY = re.compile(r'episode_([0-9]+)\Z')
SUMMARY_FIELDS = ('episode_id', 'state', 'mode', 'task', 'skill', 'outcome',
                  'started_at_unix_ns', 'saved_at_unix_ns', 'duration_limit_sec',
                  'recorded_duration_sec', 'stop_reason')


def scan_episode_directory(directory):
    """Read existing saved records only; never create paths or trust a stale index."""
    if not str(directory).strip():
        raise ValueError('请填写 Episode 保存目录')
    root = Path(directory).expanduser().resolve()
    if not root.is_dir():
        raise ValueError(f'Episode 保存目录不存在：{root}')
    episodes, warnings = [], []
    for path in root.iterdir():
        match = EPISODE_DIRECTORY.fullmatch(path.name)
        if not match or not path.is_dir():
            continue
        try:
            metadata = json.loads((path / 'episode.json').read_text(encoding='utf-8'))
            if not isinstance(metadata, dict):
                raise ValueError('episode.json 必须为对象')
            if metadata.get('state') != 'saved':
                continue
            row = {key: metadata.get(key) for key in SUMMARY_FIELDS}
            fields = metadata.get('task_metadata') or {}
            row.update(episode_number=int(match.group(1)), path=str(path),
                       language_instruction=fields.get('language_instruction') if isinstance(fields, dict) else None)
            episodes.append(row)
        except FileNotFoundError:
            # Files removed during a refresh are simply absent from the result.
            continue
        except (OSError, ValueError, UnicodeError) as error:
            warnings.append(f'{path.name}：{error}')
    episodes.sort(key=lambda row: (row['episode_number'], row['path']), reverse=True)
    return {'directory': str(root), 'episodes': episodes, 'warnings': warnings}
