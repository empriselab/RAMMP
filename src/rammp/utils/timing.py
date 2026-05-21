"""Lightweight timing instrumentation for benchmarking perception pipelines.

Usage:

    from rammp.utils.timing import timer, print_summary

    with timer("drink/color_mask"):
        mask = detect_handle_color(img)

A summary table (count, mean/min/max ms, approx max Hz) is printed
automatically at process exit, and can be printed on demand via
``print_summary()``. Instrumentation can be disabled with ``enable(False)``.
"""

import atexit
import time
from collections import defaultdict
from contextlib import contextmanager
from threading import Lock


class _Stats:
    __slots__ = ("count", "total", "min", "max", "last")

    def __init__(self) -> None:
        self.count = 0
        self.total = 0.0
        self.min = float("inf")
        self.max = 0.0
        self.last = 0.0

    def add(self, dt: float) -> None:
        self.count += 1
        self.total += dt
        self.min = min(self.min, dt)
        self.max = max(self.max, dt)
        self.last = dt


_stats: dict[str, _Stats] = defaultdict(_Stats)
_lock = Lock()
_enabled = True


def enable(flag: bool = True) -> None:
    """Globally enable/disable timing (disabled => near-zero overhead)."""
    global _enabled
    _enabled = flag


def record(label: str, dt: float) -> None:
    """Record a single elapsed-time sample (seconds) for ``label``."""
    if not _enabled:
        return
    with _lock:
        _stats[label].add(dt)


@contextmanager
def timer(label: str):
    """Context manager that records wall-clock time spent in the block."""
    if not _enabled:
        yield
        return
    start = time.perf_counter()
    try:
        yield
    finally:
        record(label, time.perf_counter() - start)


def reset() -> None:
    """Clear all accumulated stats (e.g. to separate before/after runs)."""
    with _lock:
        _stats.clear()


def summary() -> str:
    """Return a formatted table of all recorded sections, slowest first."""
    with _lock:
        items = sorted(_stats.items(), key=lambda kv: -kv[1].total)
    if not items:
        return "[timing] no samples recorded"
    lines = [
        "",
        "=" * 86,
        "PERCEPTION TIMING SUMMARY",
        "-" * 86,
        f"{'section':<32}{'n':>6}{'mean ms':>11}{'min ms':>10}"
        f"{'max ms':>10}{'total s':>10}{'~max Hz':>9}",
        "-" * 86,
    ]
    for label, s in items:
        mean_ms = s.total / s.count * 1e3
        hz = 1e3 / mean_ms if mean_ms > 0 else float("inf")
        lines.append(
            f"{label:<32}{s.count:>6}{mean_ms:>11.2f}{s.min * 1e3:>10.2f}"
            f"{s.max * 1e3:>10.2f}{s.total:>10.2f}{hz:>9.1f}"
        )
    lines.append("=" * 86)
    return "\n".join(lines)


def print_summary() -> None:
    """Print the timing summary table to stdout."""
    print(summary(), flush=True)


atexit.register(print_summary)
