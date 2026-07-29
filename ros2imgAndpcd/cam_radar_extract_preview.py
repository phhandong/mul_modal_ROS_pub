#!/usr/bin/env python3
"""Extract synchronized camera/radar data and render lightweight preview videos."""

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml
from rosbags.highlevel import AnyReader


IMAGE_TOPIC = "/sync/image_visible"
RADAR_TOPIC = "/sync/clusters"
CAMERA_SIZE = (960, 540)
RADAR_SIZE = (720, 540)


def stamp_ns(message) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def timestamp_name(timestamp_ns: int) -> str:
    return f"{timestamp_ns / 1e9:.6f}".replace(".", "_")


def radar_points(message) -> np.ndarray:
    points = [
        [
            cluster.position.pose.position.x,
            cluster.position.pose.position.y,
            cluster.position.pose.position.z,
            cluster.rcs,
        ]
        for cluster in message.clusters
    ]
    return np.asarray(points, dtype=np.float32).reshape((-1, 4))


def write_ascii_pcd(path: Path, points: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA ascii\n"
    )
    with path.open("w", encoding="ascii") as handle:
        handle.write(header)
        np.savetxt(handle, points, fmt="%.6f")


def radar_birdseye(points: np.ndarray) -> np.ndarray:
    width, height = RADAR_SIZE
    canvas = np.full((height, width, 3), 18, dtype=np.uint8)

    # Automotive convention: x is forward, y is lateral.
    x_min, x_max = 0.0, 100.0
    y_min, y_max = -30.0, 30.0
    for distance in range(0, 101, 10):
        py = int((x_max - distance) / (x_max - x_min) * (height - 1))
        cv2.line(canvas, (0, py), (width - 1, py), (48, 48, 48), 1)
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
    cv2.line(canvas, (width // 2, 0), (width // 2, height - 1), (70, 70, 70), 1)

    if len(points):
        valid = (
            (points[:, 0] >= x_min)
            & (points[:, 0] <= x_max)
            & (points[:, 1] >= y_min)
            & (points[:, 1] <= y_max)
        )
        visible = points[valid]
        if len(visible):
            px = ((y_max - visible[:, 1]) / (y_max - y_min) * (width - 1)).astype(
                np.int32
            )
            py = ((x_max - visible[:, 0]) / (x_max - x_min) * (height - 1)).astype(
                np.int32
            )
            rcs = visible[:, 3]
            lo, hi = np.percentile(rcs, [5, 95])
            values = np.clip((rcs - lo) / max(hi - lo, 1e-6) * 255, 0, 255).astype(
                np.uint8
            )
            colors = cv2.applyColorMap(values.reshape(-1, 1), cv2.COLORMAP_TURBO)[:, 0]
            for x_pixel, y_pixel, color in zip(px, py, colors):
                cv2.circle(
                    canvas,
                    (int(x_pixel), int(y_pixel)),
                    5,
                    tuple(int(value) for value in color),
                    -1,
                    cv2.LINE_AA,
                )

    radar_origin = (width // 2, height - 1)
    cv2.circle(canvas, radar_origin, 6, (255, 255, 255), -1)
    cv2.putText(
        canvas,
        f"ARS-40X clusters: {len(points)}",
        (12, height - 16),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (235, 235, 235),
        1,
        cv2.LINE_AA,
    )
    return canvas


def process_bag(source: Path, output_dir: Path, preview_path: Path) -> tuple[int, float]:
    image_dir = output_dir / "image"
    radar_dir = output_dir / "radar"
    image_dir.mkdir(parents=True, exist_ok=True)
    radar_dir.mkdir(parents=True, exist_ok=True)

    with AnyReader([source]) as reader:
        image_conn = next((c for c in reader.connections if c.topic == IMAGE_TOPIC), None)
        radar_conn = next((c for c in reader.connections if c.topic == RADAR_TOPIC), None)
        if image_conn is None or radar_conn is None:
            raise ValueError(f"{source.name}: camera or radar topic is missing")

        images = list(reader.messages(connections=[image_conn]))
        radars = list(reader.messages(connections=[radar_conn]))
        if len(images) != len(radars):
            raise ValueError(
                f"{source.name}: image/radar counts differ ({len(images)} vs {len(radars)})"
            )
        count = len(images)
        duration = max((images[-1][1] - images[0][1]) / 1e9, 0.1)
        fps = float(np.clip((count - 1) / duration, 1.0, 15.0))

        preview_path.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(
            str(preview_path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            fps,
            (CAMERA_SIZE[0] + RADAR_SIZE[0], CAMERA_SIZE[1]),
        )
        if not writer.isOpened():
            raise RuntimeError(f"could not create {preview_path}")

        timestamps = []
        try:
            for index, (image_record, radar_record) in enumerate(zip(images, radars)):
                image_message = reader.deserialize(image_record[2], image_conn.msgtype)
                radar_message = reader.deserialize(radar_record[2], radar_conn.msgtype)
                image = cv2.imdecode(
                    np.frombuffer(image_message.data, dtype=np.uint8), cv2.IMREAD_COLOR
                )
                if image is None:
                    raise ValueError(f"{source.name}: failed to decode image {index}")
                points = radar_points(radar_message)
                timestamp = stamp_ns(radar_message)
                name = timestamp_name(timestamp)

                cv2.imwrite(str(image_dir / f"{name}.png"), image)
                write_ascii_pcd(radar_dir / f"{name}.pcd", points)
                timestamps.append(
                    {
                        "frame_id": index,
                        "timestamp": timestamp / 1e9,
                        "image_radar_record_delta_ms": abs(
                            image_record[1] - radar_record[1]
                        )
                        / 1e6,
                    }
                )

                camera_preview = cv2.resize(image, CAMERA_SIZE, interpolation=cv2.INTER_AREA)
                cv2.putText(
                    camera_preview,
                    f"{source.name}  frame {index + 1}/{count}",
                    (18, CAMERA_SIZE[1] - 18),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.68,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                writer.write(np.hstack((camera_preview, radar_birdseye(points))))
        finally:
            writer.release()

    with (output_dir / "timestamps.yaml").open("w", encoding="utf-8") as handle:
        yaml.safe_dump({"frames": timestamps}, handle, allow_unicode=True, sort_keys=False)
    return count, fps


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_dir", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--previews", type=Path, required=True)
    args = parser.parse_args()

    bags = sorted(args.input_dir.glob("*.bag"))
    if not bags:
        raise SystemExit(f"no .bag files found in {args.input_dir}")
    for bag in bags:
        count, fps = process_bag(
            bag,
            args.output / bag.stem,
            args.previews / f"{bag.stem}_camera_radar_preview.mp4",
        )
        print(f"{bag.name}: extracted {count} pairs; preview {fps:.2f} fps")


if __name__ == "__main__":
    main()
