#!/usr/bin/env python3
"""Manually tune a 3-DOF radar translation against one extracted LiDAR frame."""

import argparse
import select
import sys
import termios
import time
import tty
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

from play_extracted_lidar_radar import paired_files, read_xyzi_pcd


FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]


def pointcloud_message(node: Node, points: np.ndarray) -> PointCloud2:
    packed = np.asarray(points, dtype="<f4").reshape((-1, 4))
    message = PointCloud2()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = "rslidar"
    message.height = 1
    message.width = len(packed)
    message.fields = FIELDS
    message.is_bigendian = False
    message.point_step = 16
    message.row_step = len(packed) * 16
    message.data = packed.tobytes()
    message.is_dense = bool(np.isfinite(packed[:, :3]).all())
    return message


class CalibrationPublisher(Node):
    def __init__(self) -> None:
        super().__init__("manual_lidar_radar_translation")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.lidar_publisher = self.create_publisher(
            PointCloud2, "/calib/lidar", qos
        )
        self.original_radar_publisher = self.create_publisher(
            PointCloud2, "/calib/radar_original", qos
        )
        self.adjusted_radar_publisher = self.create_publisher(
            PointCloud2, "/calib/radar_adjusted", qos
        )

    def publish_static(self, lidar: np.ndarray, radar: np.ndarray) -> None:
        self.lidar_publisher.publish(pointcloud_message(self, lidar))
        self.original_radar_publisher.publish(pointcloud_message(self, radar))

    def publish_adjusted(
        self,
        radar: np.ndarray,
        translation: np.ndarray,
    ) -> None:
        adjusted = radar.copy()
        adjusted[:, :3] += translation
        self.adjusted_radar_publisher.publish(pointcloud_message(self, adjusted))


def write_current(
    output_dir: Path,
    sequence: str,
    frame_index: int,
    lidar_path: Path,
    radar_path: Path,
    translation: np.ndarray,
    step: float,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    data = {
        "transform": "radar_to_rslidar",
        "degrees_of_freedom": "translation_only",
        "translation_unit": "meter",
        "sequence": sequence,
        "frame_index": frame_index,
        "lidar_pcd": str(lidar_path),
        "radar_pcd": str(radar_path),
        "translation": {
            "x": float(translation[0]),
            "y": float(translation[1]),
            "z": float(translation[2]),
        },
        "step": float(step),
    }
    (output_dir / "CURRENT_TRANSLATION.yaml").write_text(
        yaml.safe_dump(data, sort_keys=False),
        encoding="utf-8",
    )
    (output_dir / "CURRENT_TRANSLATION.txt").write_text(
        f"{translation[0]:.9f} {translation[1]:.9f} {translation[2]:.9f}\n",
        encoding="utf-8",
    )


def save_result(
    output_dir: Path,
    sequence: str,
    frame_index: int,
    lidar_path: Path,
    radar_path: Path,
    translation: np.ndarray,
    step: float,
) -> Path:
    write_current(
        output_dir,
        sequence,
        frame_index,
        lidar_path,
        radar_path,
        translation,
        step,
    )
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    target = output_dir / (
        f"radar_to_rslidar_seq{sequence}_frame{frame_index:03d}_{timestamp}.yaml"
    )
    target.write_text(
        (output_dir / "CURRENT_TRANSLATION.yaml").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    return target


def print_state(translation: np.ndarray, step: float) -> None:
    print(
        "\r"
        f"translation [m]  x={translation[0]: .4f}  "
        f"y={translation[1]: .4f}  z={translation[2]: .4f}  "
        f"step={step:.4f}     ",
        end="",
        flush=True,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("sequence_dir", type=Path)
    parser.add_argument("--frame", type=int, default=0)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--step", type=float, default=0.01)
    parser.add_argument("--initial-x", type=float, default=0.0)
    parser.add_argument("--initial-y", type=float, default=0.0)
    parser.add_argument("--initial-z", type=float, default=0.0)
    parser.add_argument("--load", type=Path)
    parser.add_argument("--max-lidar-points", type=int, default=120_000)
    args = parser.parse_args()

    pairs = paired_files(args.sequence_dir)
    if not 0 <= args.frame < len(pairs):
        raise SystemExit(f"--frame must be between 0 and {len(pairs) - 1}")
    if args.step <= 0:
        raise SystemExit("--step must be positive")

    lidar_path, radar_path = pairs[args.frame]
    lidar = read_xyzi_pcd(lidar_path)
    radar = read_xyzi_pcd(radar_path)
    if args.max_lidar_points > 0 and len(lidar) > args.max_lidar_points:
        stride = int(np.ceil(len(lidar) / args.max_lidar_points))
        lidar = lidar[::stride]
    initial_translation = [args.initial_x, args.initial_y, args.initial_z]
    step = float(args.step)
    if args.load and args.load.is_file():
        loaded = yaml.safe_load(args.load.read_text(encoding="utf-8"))
        saved = loaded["translation"]
        initial_translation = [saved["x"], saved["y"], saved["z"]]
    translation = np.asarray(initial_translation, dtype=np.float32)

    rclpy.init()
    node = CalibrationPublisher()
    time.sleep(0.5)
    node.publish_static(lidar, radar)
    node.publish_adjusted(radar, translation)
    write_current(
        args.output_dir,
        args.sequence_dir.name,
        args.frame,
        lidar_path,
        radar_path,
        translation,
        step,
    )

    print(
        "\nManual radar translation controls:\n"
        "  w/s: +x/-x    a/d: +y/-y    q/e: +z/-z\n"
        "  +/-: increase/decrease step    r: reset\n"
        "  k: save final YAML              ESC: quit\n"
    )
    print_state(translation, step)
    old_terminal = termios.tcgetattr(sys.stdin.fileno())
    tty.setcbreak(sys.stdin.fileno())
    dirty = False
    last_adjusted_publish = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if time.monotonic() - last_adjusted_publish >= 0.1:
                node.publish_adjusted(radar, translation)
                last_adjusted_publish = time.monotonic()
            ready, _, _ = select.select([sys.stdin], [], [], 0.03)
            if not ready:
                continue
            key = sys.stdin.read(1)
            if key == "\x1b":
                break
            if key == "w":
                translation[0] += step
            elif key == "s":
                translation[0] -= step
            elif key == "a":
                translation[1] += step
            elif key == "d":
                translation[1] -= step
            elif key == "q":
                translation[2] += step
            elif key == "e":
                translation[2] -= step
            elif key in ("+", "="):
                step *= 10.0
            elif key in ("-", "_"):
                step = max(step / 10.0, 0.0001)
            elif key == "r":
                translation[:] = 0.0
            elif key == "k":
                target = save_result(
                    args.output_dir,
                    args.sequence_dir.name,
                    args.frame,
                    lidar_path,
                    radar_path,
                    translation,
                    step,
                )
                print(f"\nSaved: {target}")
                print_state(translation, step)
                continue
            else:
                continue
            dirty = True

            if dirty:
                node.publish_adjusted(radar, translation)
                last_adjusted_publish = time.monotonic()
                write_current(
                    args.output_dir,
                    args.sequence_dir.name,
                    args.frame,
                    lidar_path,
                    radar_path,
                    translation,
                    step,
                )
                print_state(translation, step)
                dirty = False
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_terminal)
        print()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
