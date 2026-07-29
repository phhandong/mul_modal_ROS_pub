#!/usr/bin/env python3
"""Render synchronized camera and LiDAR ROS bag data as lightweight MP4 previews."""

import argparse
from pathlib import Path

import cv2
import numpy as np
from rosbags.highlevel import AnyReader

from pcd_io import pointcloud_xyzi


IMAGE_TOPIC = "/sync/image_visible"
POINTCLOUD_TOPIC = "/sync/pointcloud"
CAMERA_SIZE = (960, 540)
LIDAR_SIZE = (720, 540)


def lidar_birdseye(message, max_points: int = 35_000) -> np.ndarray:
    width, height = LIDAR_SIZE
    canvas = np.full((height, width, 3), 18, dtype=np.uint8)
    points = pointcloud_xyzi(message)
    if len(points) > max_points:
        points = points[:: max(1, len(points) // max_points)]

    # Forward x: -10..80 m; lateral y: -45..45 m.
    mask = (
        (points[:, 0] >= -10.0)
        & (points[:, 0] <= 80.0)
        & (points[:, 1] >= -45.0)
        & (points[:, 1] <= 45.0)
    )
    points = points[mask]

    for distance in range(0, 81, 10):
        py = int((80.0 - distance) / 90.0 * (height - 1))
        cv2.line(canvas, (0, py), (width - 1, py), (50, 50, 50), 1)
        cv2.putText(
            canvas,
            f"{distance} m",
            (8, max(16, py - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (150, 150, 150),
            1,
            cv2.LINE_AA,
        )
    cv2.line(canvas, (width // 2, 0), (width // 2, height - 1), (65, 65, 65), 1)

    if len(points):
        px = ((45.0 - points[:, 1]) / 90.0 * (width - 1)).astype(np.int32)
        py = ((80.0 - points[:, 0]) / 90.0 * (height - 1)).astype(np.int32)
        intensity = np.nan_to_num(points[:, 3], nan=0.0)
        lo, hi = np.percentile(intensity, [2, 98])
        normalized = np.clip((intensity - lo) / max(hi - lo, 1e-6) * 255, 0, 255).astype(
            np.uint8
        )
        colors = cv2.applyColorMap(normalized.reshape(-1, 1), cv2.COLORMAP_TURBO)[:, 0]
        canvas[py, px] = colors

    cv2.circle(canvas, (width // 2, int(80.0 / 90.0 * (height - 1))), 5, (255, 255, 255), -1)
    cv2.putText(
        canvas,
        f"LiDAR bird's-eye  points: {len(points):,}",
        (12, height - 14),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (235, 235, 235),
        1,
        cv2.LINE_AA,
    )
    return canvas


def render_bag(source: Path, output: Path) -> tuple[int, float]:
    with AnyReader([source]) as reader:
        image_conn = next(c for c in reader.connections if c.topic == IMAGE_TOPIC)
        cloud_conn = next(c for c in reader.connections if c.topic == POINTCLOUD_TOPIC)
        images = list(reader.messages(connections=[image_conn]))
        clouds = list(reader.messages(connections=[cloud_conn]))
        count = min(len(images), len(clouds))
        duration = max((images[count - 1][1] - images[0][1]) / 1e9, 0.1)
        fps = float(np.clip((count - 1) / duration, 1.0, 15.0))

        output.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(
            str(output),
            cv2.VideoWriter_fourcc(*"mp4v"),
            fps,
            (CAMERA_SIZE[0] + LIDAR_SIZE[0], CAMERA_SIZE[1]),
        )
        if not writer.isOpened():
            raise RuntimeError(f"could not create video: {output}")

        try:
            for index in range(count):
                image_msg = reader.deserialize(images[index][2], image_conn.msgtype)
                cloud_msg = reader.deserialize(clouds[index][2], cloud_conn.msgtype)
                camera = cv2.imdecode(
                    np.frombuffer(image_msg.data, dtype=np.uint8), cv2.IMREAD_COLOR
                )
                if camera is None:
                    raise ValueError(f"{source.name}: image {index} could not be decoded")
                camera = cv2.resize(camera, CAMERA_SIZE, interpolation=cv2.INTER_AREA)
                cv2.putText(
                    camera,
                    f"{source.name}  frame {index + 1}/{count}",
                    (18, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                writer.write(np.hstack((camera, lidar_birdseye(cloud_msg))))
        finally:
            writer.release()
    return count, fps


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_dir", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    bags = sorted(args.input_dir.glob("*.bag"), key=lambda path: int(path.stem))
    if not bags:
        raise SystemExit(f"no .bag files found in {args.input_dir}")
    for bag in bags:
        target = args.output / f"{bag.stem}_lidar_cam_preview.mp4"
        count, fps = render_bag(bag, target)
        print(f"{bag.name}: {count} synchronized frames, {fps:.2f} fps -> {target}")


if __name__ == "__main__":
    main()
