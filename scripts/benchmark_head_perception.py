#!/usr/bin/env python3
"""Benchmark head perception in a tight loop, without the action server.

Drives ``PerceptionInterface.run_head_perception()`` as fast as possible
against the live RealSense feed, then prints the timing summary. The same
script works at both commits used for the before/after comparison because the
API it touches — ``PerceptionInterface.run_head_perception()`` — is unchanged
between them:

    - "after" (branch tip):       MediaPipeHeadPerception under the hood
    - "before" (commit 9cd9042):  the original DECA HeadPerception

To benchmark "before" without losing this file, copy it outside the working
tree before checking out the old commit, then run from the copy:

    cp scripts/benchmark_head_perception.py /tmp/
    git checkout 9cd9042
    python3 /tmp/benchmark_head_perception.py
    git checkout feature/improve-performance

Prerequisites:
    - A live RealSense camera publishing the wrist camera topics.
    - Head-perception calibration set up for the current commit:
        * commit 9cd9042: the committed DECA calibration is used out of the box.
        * branch tip:     run ``calibrate_head`` once on the robot first.
    - The Python env appropriate to the current commit (DECA stack at 9cd9042;
      MediaPipe at the branch tip).
"""

import argparse
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rammp.interfaces.perception_interface import PerceptionInterface
from rammp.utils.timing import print_summary, reset


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Benchmark head perception in a tight loop."
    )
    parser.add_argument(
        "--iterations",
        type=int,
        default=200,
        help="Number of perception calls to time (default: 200).",
    )
    parser.add_argument(
        "--warmup",
        type=int,
        default=10,
        help="Warmup iterations excluded from timing (default: 10).",
    )
    args = parser.parse_args()

    rclpy.init()
    node = Node("head_perception_benchmark")
    perception = PerceptionInterface(node=node, simulation=False, log_dir=None)

    # PerceptionInterface.__init__ runs ~10 warm-start iterations on stale
    # data while it's waiting for camera frames. Drop those samples.
    reset()

    # Spin the node on a background thread so the RealSense subscriber
    # callbacks keep delivering fresh frames during the benchmark loop.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    # Give the executor a moment to begin processing callbacks.
    time.sleep(0.5)

    # Warm-up against the live feed (excluded from the recorded timings).
    for _ in range(args.warmup):
        perception.run_head_perception()
    reset()

    print(
        f"Benchmarking {args.iterations} head-perception iterations "
        f"(after {args.warmup} live warmup iterations)..."
    )
    successes = 0
    t_start = time.perf_counter()
    for _ in range(args.iterations):
        result = perception.run_head_perception()
        if result is not None:
            successes += 1
    t_elapsed = time.perf_counter() - t_start

    print(
        f"\n{args.iterations} iterations in {t_elapsed:.2f}s "
        f"=> {args.iterations / t_elapsed:.1f} Hz wall-clock"
    )
    print(f"  successful detections: {successes}/{args.iterations}")
    print_summary()

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
