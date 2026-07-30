"""Signal-safe rosbag2 wrapper used by the ARS408 launch file."""

import signal
import subprocess
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node


EXPECTED_TOPICS = (
    "/ars408/points_raw",
    "/ars408/points",
    "/ars408/status",
)
PUBLISHER_WAIT_SECONDS = 10.0


def duplicate_publisher_error(counts: dict[str, int]) -> Optional[str]:
    """Describe duplicate publishers, or return None when counts are safe."""
    duplicates = [
        f"{topic}={count}"
        for topic, count in counts.items()
        if count > 1
    ]
    if not duplicates:
        return None
    return (
        "refusing to record because another radar/playback/filter "
        f"publisher is active: {', '.join(duplicates)}"
    )


def publishers_ready(counts: dict[str, int]) -> bool:
    """Return whether each required data topic has exactly one publisher."""
    return (
        set(counts) == set(EXPECTED_TOPICS)
        and all(count == 1 for count in counts.values())
    )


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: ars408_bag_recorder OUTPUT_DIRECTORY")

    command = [
        "ros2", "bag", "record",
        "--storage", "mcap",
        "--storage-preset-profile", "zstd_fast",
        "--output", sys.argv[1],
        "--topics",
        "/ars408/points_raw",
        "/ars408/points",
        "/ars408/status",
        "/tf",
        "/tf_static",
        "/parameter_events",
    ]
    process: Optional[subprocess.Popen] = None
    stopping = False

    def stop(_signum, _frame) -> None:
        nonlocal stopping
        stopping = True
        if process is not None and process.poll() is None:
            # rosbag2 reliably flushes MCAP metadata on SIGTERM in a
            # non-interactive launch process.
            process.terminate()

    rclpy.init()
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    guard = Node("ars408_bag_recorder_guard")

    def publisher_counts() -> dict[str, int]:
        return {
            topic: len(guard.get_publishers_info_by_topic(topic))
            for topic in EXPECTED_TOPICS
        }

    try:
        deadline = time.monotonic() + PUBLISHER_WAIT_SECONDS
        while not stopping:
            counts = publisher_counts()
            error = duplicate_publisher_error(counts)
            if error:
                raise RuntimeError(error)
            if publishers_ready(counts):
                break
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "timed out waiting for one ARS408 publisher on each "
                    f"recorded data topic: {counts}"
                )
            rclpy.spin_once(guard, timeout_sec=0.1)

        if stopping:
            return

        process = subprocess.Popen(command)
        while process.poll() is None and not stopping:
            counts = publisher_counts()
            error = duplicate_publisher_error(counts)
            if error:
                guard.get_logger().error(error)
                process.terminate()
                break
            if any(count == 0 for count in counts.values()):
                guard.get_logger().error(
                    "ARS408 publisher disappeared while recording: "
                    f"{counts}"
                )
                process.terminate()
                break
            rclpy.spin_once(guard, timeout_sec=0.25)
        exit_code = process.wait()
        if exit_code and not stopping:
            raise SystemExit(exit_code)
    except RuntimeError as exc:
        guard.get_logger().error(str(exc))
        raise SystemExit(2) from exc
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            process.wait()
        guard.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
