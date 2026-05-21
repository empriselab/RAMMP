import threading

from rammp.perception.gestures_perception.static_gesture_detectors import mouth_open


class _FakePerception:
    """Yields a preset sequence of head-perception dicts on each call."""

    def __init__(self, sequence):
        self._sequence = list(sequence)
        self._calls = 0

    def run_head_perception(self):
        if self._calls < len(self._sequence):
            value = self._sequence[self._calls]
        else:
            value = self._sequence[-1]
        self._calls += 1
        return value


def test_mouth_open_returns_true_when_jaw_open():
    perception = _FakePerception([{"jaw_open_score": 0.9}])
    assert mouth_open(perception, termination_event=None, timeout=5) is True


def test_mouth_open_skips_none_then_detects():
    perception = _FakePerception([None, None, {"jaw_open_score": 0.8}])
    assert mouth_open(perception, termination_event=None, timeout=5) is True


def test_mouth_open_returns_false_when_terminated():
    event = threading.Event()
    event.set()
    perception = _FakePerception([{"jaw_open_score": 0.0}])
    assert mouth_open(perception, termination_event=event, timeout=5) is False


def test_mouth_open_returns_false_on_timeout():
    perception = _FakePerception([{"jaw_open_score": 0.1}])
    assert mouth_open(perception, termination_event=None, timeout=0.2) is False
