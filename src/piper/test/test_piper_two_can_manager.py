"""Exercise service confirmations and the live dual-side barrier offline."""

import threading
import time

import pytest

pytest.importorskip('piper_msgs.msg')

from piper.piper_two_can_align import SafetyError  # noqa: E402
from piper.piper_two_can_manager import (  # noqa: E402
    ActiveBarrier, ServiceOperator,
)


def test_aborting_wait_is_never_a_human_confirmation():
    operator = ServiceOperator('left', lambda _: None)
    operator.abort()
    assert operator.wait_teach_engaged(1) is False
    assert operator.wait_teach_released(1) is False


def test_stop_interrupts_wait_for_teach():
    operator = ServiceOperator('left', lambda _: None)
    operator.request_stop('stop')
    with pytest.raises(KeyboardInterrupt):
        operator.wait_teach_engaged(1)
    assert operator.wait_teach_released(0.01) is False


def test_barrier_keeps_receiving_while_peer_is_delayed():
    barrier = ActiveBarrier(timeout_s=1)
    received = threading.Event()
    completed = []

    def receive():
        received.set()
        time.sleep(0.001)

    def first():
        barrier.wait(receive)
        completed.append(True)

    thread = threading.Thread(target=first)
    thread.start()
    assert received.wait(0.5)
    barrier.wait(lambda: None)
    thread.join(0.5)
    assert completed == [True]


def test_barrier_timeout_remains_bounded():
    barrier = ActiveBarrier(timeout_s=0.01)
    with pytest.raises(SafetyError, match='超时'):
        barrier.wait(lambda: time.sleep(0.001))
