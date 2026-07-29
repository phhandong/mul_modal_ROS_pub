#!/usr/bin/env python3
"""Extract synchronized LiDAR/radar PCDs and render bird's-eye preview videos."""

import argparse
from io import BytesIO
from pathlib import Path

import cv2
import numpy as np
import yaml
from rosbags.highlevel import AnyReader
from rosbags.rosbag1.reader import (
    Header,
    RecordType,
    decompressors,
    read_bytes,
    read_uint32,
)
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from rosbags.typesys.msg import normalize_msgtype

from pcd_io import pointcloud_xyzi, write_pcd


LIDAR_TOPIC = "/sync/pointcloud"
RADAR_TOPIC = "/sync/clusters"
PANEL_SIZE = (840, 540)


def stamp_ns(message) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def timestamp_name(timestamp_ns: int) -> str:
    return f"{timestamp_ns / 1e9:.6f}".replace(".", "_")


def radar_points(message) -> np.ndarray:
    return np.asarray(
        [
            [
                cluster.position.pose.position.x,
                cluster.position.pose.position.y,
                cluster.position.pose.position.z,
                cluster.rcs,
            ]
            for cluster in message.clusters
        ],
        dtype=np.float32,
    ).reshape((-1, 4))


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


def birdseye(
    points: np.ndarray,
    *,
    title: str,
    large_marks: bool,
    max_points: int = 40_000,
) -> np.ndarray:
    width, height = PANEL_SIZE
    canvas = np.full((height, width, 3), 18, dtype=np.uint8)
    if len(points) > max_points:
        step = int(np.ceil(len(points) / max_points))
        points = points[::step]

    x_min, x_max = -10.0, 100.0
    y_min, y_max = -40.0, 40.0
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

    visible = points[
        (points[:, 0] >= x_min)
        & (points[:, 0] <= x_max)
        & (points[:, 1] >= y_min)
        & (points[:, 1] <= y_max)
    ]
    if len(visible):
        px = ((y_max - visible[:, 1]) / (y_max - y_min) * (width - 1)).astype(np.int32)
        py = ((x_max - visible[:, 0]) / (x_max - x_min) * (height - 1)).astype(np.int32)
        values = np.nan_to_num(visible[:, 3], nan=0.0)
        lo, hi = np.percentile(values, [3, 97])
        colors = cv2.applyColorMap(
            np.clip((values - lo) / max(hi - lo, 1e-6) * 255, 0, 255)
            .astype(np.uint8)
            .reshape(-1, 1),
            cv2.COLORMAP_TURBO,
        )[:, 0]
        if large_marks:
            for x_pixel, y_pixel, color in zip(px, py, colors):
                cv2.circle(
                    canvas,
                    (int(x_pixel), int(y_pixel)),
                    5,
                    tuple(int(value) for value in color),
                    -1,
                    cv2.LINE_AA,
                )
        else:
            canvas[py, px] = colors

    origin_y = int((x_max - 0.0) / (x_max - x_min) * (height - 1))
    cv2.circle(canvas, (width // 2, origin_y), 5, (255, 255, 255), -1)
    cv2.putText(
        canvas,
        f"{title}  points: {len(points):,}",
        (12, height - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (235, 235, 235),
        1,
        cv2.LINE_AA,
    )
    return canvas


def iter_normal_pairs(path: Path):
    with AnyReader([path]) as reader:
        lidar_conn = next(c for c in reader.connections if c.topic == LIDAR_TOPIC)
        radar_conn = next(c for c in reader.connections if c.topic == RADAR_TOPIC)
        current = {}
        for connection, timestamp, raw in reader.messages(
            connections=[lidar_conn, radar_conn]
        ):
            message = reader.deserialize(raw, connection.msgtype)
            current[connection.topic] = (timestamp, message)
            if LIDAR_TOPIC in current and RADAR_TOPIC in current:
                yield current[LIDAR_TOPIC], current[RADAR_TOPIC]
                current = {}


def iter_salvaged_pairs(path: Path):
    """Read complete records from a ROS1 bag prefix without relying on its index."""
    typestore = get_typestore(Stores.ROS1_NOETIC)
    connections = {}
    current = {}

    with path.open("rb") as source:
        if source.readline() != b"#ROSBAG V2.0\n":
            raise ValueError("missing ROSBAG V2.0 header")
        Header.read(source, RecordType.BAGHEADER)
        source.seek(read_uint32(source), 1)

        while source.tell() < path.stat().st_size:
            try:
                outer = Header.read(source)
                operation = outer.get_uint8("op")
                if operation == RecordType.CHUNK:
                    compressed = read_bytes(source, read_uint32(source))
                    raw_chunk = decompressors[outer.get_string("compression")](compressed)
                    chunk = BytesIO(raw_chunk)
                    while chunk.tell() < len(raw_chunk):
                        header = Header.read(chunk)
                        inner_operation = header.get_uint8("op")
                        if inner_operation == RecordType.CONNECTION:
                            definition = Header.read(chunk)
                            connection_id = header.get_uint32("conn")
                            topic = header.get_string("topic")
                            message_type = normalize_msgtype(definition.get_string("type"))
                            typestore.register(
                                get_types_from_msg(
                                    definition.get_string("message_definition"), message_type
                                )
                            )
                            connections[connection_id] = (topic, message_type)
                        elif inner_operation == RecordType.MSGDATA:
                            raw_message = read_bytes(chunk, read_uint32(chunk))
                            connection_id = header.get_uint32("conn")
                            if connection_id not in connections:
                                continue
                            topic, message_type = connections[connection_id]
                            if topic not in (LIDAR_TOPIC, RADAR_TOPIC):
                                continue
                            timestamp = header.get_time("time")
                            message = typestore.deserialize_ros1(raw_message, message_type)
                            current[topic] = (timestamp, message)
                            if LIDAR_TOPIC in current and RADAR_TOPIC in current:
                                yield current[LIDAR_TOPIC], current[RADAR_TOPIC]
                                current = {}
                        else:
                            raise ValueError(f"unsupported inner record {inner_operation}")
                elif operation == RecordType.IDXDATA:
                    source.seek(read_uint32(source), 1)
                elif operation == RecordType.CONNECTION:
                    Header.read(source)
                elif operation == RecordType.CHUNK_INFO:
                    source.seek(read_uint32(source), 1)
                else:
                    raise ValueError(f"unsupported outer record {operation}")
            except Exception:
                # A truncated/corrupt suffix is expected in recovery mode. Only
                # complete synchronized pairs yielded before it are retained.
                break


def process_bag(
    source: Path,
    output_dir: Path,
    preview_path: Path,
    *,
    recover: bool,
) -> tuple[int, float]:
    lidar_dir = output_dir / "lidar"
    radar_dir = output_dir / "radar"
    lidar_dir.mkdir(parents=True, exist_ok=True)
    radar_dir.mkdir(parents=True, exist_ok=True)
    preview_path.parent.mkdir(parents=True, exist_ok=True)

    pairs = iter_salvaged_pairs(source) if recover else iter_normal_pairs(source)
    writer = cv2.VideoWriter(
        str(preview_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        7.0,
        (PANEL_SIZE[0] * 2, PANEL_SIZE[1]),
    )
    if not writer.isOpened():
        raise RuntimeError(f"could not create {preview_path}")

    timestamps = []
    count = 0
    first_record_time = last_record_time = None
    try:
        for lidar_item, radar_item in pairs:
            lidar_record_time, lidar_message = lidar_item
            radar_record_time, radar_message = radar_item
            lidar = pointcloud_xyzi(lidar_message)
            radar = radar_points(radar_message)
            timestamp = stamp_ns(lidar_message)
            name = timestamp_name(timestamp)
            write_pcd(lidar_dir / f"{name}.pcd", lidar, binary=True)
            write_ascii_pcd(radar_dir / f"{name}.pcd", radar)
            timestamps.append(
                {
                    "frame_id": count,
                    "timestamp": timestamp / 1e9,
                    "lidar_radar_record_delta_ms": abs(
                        lidar_record_time - radar_record_time
                    )
                    / 1e6,
                }
            )
            writer.write(
                np.hstack(
                    (
                        birdseye(lidar, title="RS LiDAR", large_marks=False),
                        birdseye(radar, title="ARS-40X radar", large_marks=True),
                    )
                )
            )
            first_record_time = (
                lidar_record_time if first_record_time is None else first_record_time
            )
            last_record_time = lidar_record_time
            count += 1
    finally:
        writer.release()

    if not count:
        raise ValueError(f"{source.name}: no complete LiDAR/radar pairs recovered")
    duration = max((last_record_time - first_record_time) / 1e9, 0.1)
    fps = float(np.clip((count - 1) / duration, 1.0, 15.0))

    # Rewrite at the observed rate so playback duration matches the bag.
    capture = cv2.VideoCapture(str(preview_path))
    frames = []
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        frames.append(frame)
    capture.release()
    writer = cv2.VideoWriter(
        str(preview_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (PANEL_SIZE[0] * 2, PANEL_SIZE[1]),
    )
    for frame in frames:
        writer.write(frame)
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

    report = []
    for bag in sorted(args.input_dir.glob("*.bag"), key=lambda path: int(path.stem)):
        with bag.open("rb") as source:
            magic = source.read(13)
        if magic != b"#ROSBAG V2.0\n":
            report.append(f"{bag.name}: skipped; missing ROSBAG V2.0 header")
            print(report[-1])
            continue
        recover = bag.stem == "21"
        try:
            count, fps = process_bag(
                bag,
                args.output / bag.stem,
                args.previews / f"{bag.stem}_lidar_radar_preview.mp4",
                recover=recover,
            )
            qualifier = "recovered prefix" if recover else "complete"
            report.append(
                f"{bag.name}: {qualifier}; extracted {count} synchronized pairs; "
                f"preview {fps:.2f} fps"
            )
            print(report[-1])
        except Exception as error:
            report.append(f"{bag.name}: skipped; {error}")
            print(report[-1])

    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "EXTRACTION_REPORT.txt").write_text(
        "\n".join(report) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    main()
