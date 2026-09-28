#!/usr/bin/env python3
"""Backward-compatible public feedback API.

Implementations live in focused modules. Existing imports from
``piper.piper_feedback`` remain supported while callers migrate gradually.
"""

from piper.piper_feedback_decode import *  # noqa: F401,F403
from piper.piper_filters import *  # noqa: F401,F403
from piper.piper_motion import *  # noqa: F401,F403

from piper.piper_feedback_decode import __all__ as _decode_exports
from piper.piper_filters import __all__ as _filter_exports
from piper.piper_motion import __all__ as _motion_exports

__all__ = _decode_exports + _motion_exports + _filter_exports

