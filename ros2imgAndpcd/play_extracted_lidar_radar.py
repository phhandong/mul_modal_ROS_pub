#!/usr/bin/env python3
"""Publish extracted LiDAR/radar PCD pairs directly for RViz2 playback."""

import argparse
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]


def read_xyzi_pcd(path: Path) -> np.ndarray:
    metadata = {}
    with path.open("rb") as source:
        while True:
            line = source.readline()
            if not line:
                raise ValueError(f"{path}: missing DATA header")
            text = line.decode("ascii").strip()
            if text and not text.startswith("#"):
                key, *values = text.split()
                metadata[key.upper()] = values
            if text.upper().startswith("DATA "):
                mode = metadata["DATA"][0].lower()
                break
        point_count = int(metadata["POINTS"][0])
        fields = metadata["FIELDS"]
        if fields != ["x", "y", "z", "intensity"]:
            raise ValueError(f"{path}: unsupported fields {fields}")
        if mode == "binary":
            expected_bytes = point_count * 4 * np.dtype("<f4").itemsize
            payload = source.read(expected_bytes)
            if len(payload) != expected_bytes:
                raise ValueError(f"{path}: truncated binary payload")
            return np.frombuffer(payload, dtype="<f4").reshape((-1, 4)).copy()
        if mode == "ascii":
            points = np.loadtxt(source, dtype=np.float32)
            return np.asarray(points, dtype=np.float32).reshape((-1, 4))
        raise ValueError(f"{path}: unsupported DATA mode {mode}")


def pointcloud_message(node: Node, points: np.ndarray, frame_id: str) -> PointCloud2:
    packed = np.asarray(points, dtype="<f4").reshape((-1, 4))
    message = PointCloud2()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = frame_id
    message.height = 1
    message.width = len(packed)
    message.fields = FIELDS
    message.is_bigendian = False
    message.point_step = 16
    message.row_step = len(packed) * 16
    message.data = packed.tobytes()
    message.is_dense = bool(np.isfinite(packed[:, :3]).all())
    return message


def timestamp_from_name(path: Path) -> float:
    return float(path.stem.replace("_", "."))


class ExtractedPlayer(Node):
    def __init__(self) -> None:
        super().__init__("extracted_lidar_radar_player")
        pointcloud_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.lidar_publisher = self.create_publisher(
            PointCloud2,
            "/lidar/points",
            pointcloud_qos,
        )
        self.radar_publisher = self.create_publisher(
            PointCloud2,
            "/radar/points",
            pointcloud_qos,
        )

    def publish_pair(
        self,
        lidar: np.ndarray,
        radar: np.ndarray,
    ) -> None:
        # The extracted files contain no calibrated LiDAR/radar extrinsic.
        # Use the shared forward-left-up convention for visualization only.
        self.lidar_publisher.publish(pointcloud_message(self, lidar, "rslidar"))
        self.radar_publisher.publish(pointcloud_message(self, radar, "rslidar"))


def paired_files(sequence_dir: Path) -> list[tuple[Path, Path]]:
    lidar_files = sorted((sequence_dir / "lidar").glob("*.pcd"))
    pairs = []
    for lidar_path in lidar_files:
        radar_path = sequence_dir / "radar" / lidar_path.name
        if not radar_path.is_file():
            raise ValueError(f"missing matching radar PCD: {radar_path}")
        pairs.append((lidar_path, radar_path))
    if not pairs:
        raise ValueError(f"no PCD pairs found in {sequence_dir}")
    return pairs


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("sequence_dir", type=Path)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument(
        "--hz",
        type=float,
        default=0.0,
        help="fixed playback frequency; zero uses recorded timing",
    )
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--max-lidar-points", type=int, default=80_000)
    args = parser.parse_args()
    if args.rate <= 0:
        raise SystemExit("--rate must be positive")
    if args.hz < 0:
        raise SystemExit("--hz cannot be negative")

    pairs = paired_files(args.sequence_dir)
    timestamps = [timestamp_from_name(lidar) for lidar, _ in pairs]
    rclpy.init()
    node = ExtractedPlayer()
    node.get_logger().info(
        f"Playing {args.sequence_dir.name}: {len(pairs)} synchronized PCD pairs"
    )

    try:
        next_deadline = time.monotonic()
        while rclpy.ok():
            for index, ((lidar_path, radar_path), timestamp) in enumerate(
                zip(pairs, timestamps)
            ):
                lidar = read_xyzi_pcd(lidar_path)
                radar = read_xyzi_pcd(radar_path)
                if args.max_lidar_points > 0 and len(lidar) > args.max_lidar_points:
                    step = int(np.ceil(len(lidar) / args.max_lidar_points))
                    lidar = lidar[::step]
                node.publish_pair(lidar, radar)
                rclpy.spin_once(node, timeout_sec=0.0)

                if args.hz > 0:
                    next_deadline += 1.0 / args.hz
                    time.sleep(max(0.0, next_deadline - time.monotonic()))
                elif index + 1 < len(pairs):
                    delay = max(
                        0.01,
                        min(timestamps[index + 1] - timestamp, 0.5),
                    ) / args.rate
                    time.sleep(delay)
            if not args.loop:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
