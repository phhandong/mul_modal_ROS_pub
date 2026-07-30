#!/usr/bin/env python3
"""Convert ROS 1 camera/radar bags to lightweight ROS 2 MCAP bags for RViz2."""

import argparse
from pathlib import Path

import cv2
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore


SOURCE_IMAGE_TOPIC = "/sync/image_visible"
SOURCE_RADAR_TOPIC = "/sync/clusters"
OUTPUT_IMAGE_TOPIC = "/camera/image/compressed"
OUTPUT_RADAR_TOPIC = "/radar/points"


def stamp_ns(message) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def converted_image(typestore, source, horizontal_flip: bool):
    image = cv2.imdecode(
        np.frombuffer(source.data, dtype=np.uint8),
        cv2.IMREAD_COLOR,
    )
    if image is None:
        raise ValueError("failed to decode compressed camera image")
    if horizontal_flip:
        image = cv2.flip(image, 1)
    ok, encoded = cv2.imencode(
        ".jpg",
        image,
        [cv2.IMWRITE_JPEG_QUALITY, 95],
    )
    if not ok:
        raise ValueError("failed to encode camera image")

    time_cls = typestore.types["builtin_interfaces/msg/Time"]
    header_cls = typestore.types["std_msgs/msg/Header"]
    image_cls = typestore.types["sensor_msgs/msg/CompressedImage"]
    timestamp = stamp_ns(source)
    return image_cls(
        header=header_cls(
            stamp=time_cls(
                sec=timestamp // 1_000_000_000,
                nanosec=timestamp % 1_000_000_000,
            ),
            frame_id="hik_camera",
        ),
        format="jpeg",
        data=encoded.reshape(-1).astype(np.uint8, copy=False),
    )


def radar_pointcloud(typestore, source):
    points = np.asarray(
        [
            (
                cluster.position.pose.position.x,
                cluster.position.pose.position.y,
                cluster.position.pose.position.z,
                cluster.rcs,
            )
            for cluster in source.clusters
        ],
        dtype="<f4",
    ).reshape((-1, 4))

    time_cls = typestore.types["builtin_interfaces/msg/Time"]
    header_cls = typestore.types["std_msgs/msg/Header"]
    field_cls = typestore.types["sensor_msgs/msg/PointField"]
    cloud_cls = typestore.types["sensor_msgs/msg/PointCloud2"]
    timestamp = stamp_ns(source)
    fields = [
        field_cls(name=name, offset=offset, datatype=7, count=1)
        for name, offset in (("x", 0), ("y", 4), ("z", 8), ("intensity", 12))
    ]
    return cloud_cls(
        header=header_cls(
            stamp=time_cls(
                sec=timestamp // 1_000_000_000,
                nanosec=timestamp % 1_000_000_000,
            ),
            frame_id="radar",
        ),
        height=1,
        width=len(points),
        fields=fields,
        is_bigendian=False,
        point_step=16,
        row_step=len(points) * 16,
        data=points.view(np.uint8).reshape(-1),
        is_dense=True,
    )


def convert_bag(
    source: Path,
    destination: Path,
    horizontal_flip: bool,
) -> tuple[int, int, int]:
    typestore = get_typestore(Stores.ROS2_JAZZY)
    minimum_points = 2**31 - 1
    maximum_points = 0

    with AnyReader([source]) as reader:
        image_connection = next(
            (item for item in reader.connections if item.topic == SOURCE_IMAGE_TOPIC),
            None,
        )
        radar_connection = next(
            (item for item in reader.connections if item.topic == SOURCE_RADAR_TOPIC),
            None,
        )
        if image_connection is None or radar_connection is None:
            raise ValueError(f"{source.name}: camera or radar clusters are missing")

        images = list(reader.messages(connections=[image_connection]))
        radars = list(reader.messages(connections=[radar_connection]))
        if len(images) != len(radars):
            raise ValueError(
                f"{source.name}: image/radar counts differ "
                f"({len(images)} vs {len(radars)})"
            )

        with Writer(
            destination,
            version=9,
            storage_plugin=StoragePlugin.MCAP,
        ) as writer:
            output_image = writer.add_connection(
                OUTPUT_IMAGE_TOPIC,
                "sensor_msgs/msg/CompressedImage",
                typestore=typestore,
            )
            output_radar = writer.add_connection(
                OUTPUT_RADAR_TOPIC,
                "sensor_msgs/msg/PointCloud2",
                typestore=typestore,
            )

            for image_record, radar_record in zip(images, radars):
                source_image = reader.deserialize(
                    image_record[2],
                    image_connection.msgtype,
                )
                source_radar = reader.deserialize(
                    radar_record[2],
                    radar_connection.msgtype,
                )
                image_message = converted_image(
                    typestore,
                    source_image,
                    horizontal_flip,
                )
                radar_message = radar_pointcloud(typestore, source_radar)
                minimum_points = min(minimum_points, radar_message.width)
                maximum_points = max(maximum_points, radar_message.width)

                writer.write(
                    output_image,
                    image_record[1],
                    typestore.serialize_cdr(
                        image_message,
                        "sensor_msgs/msg/CompressedImage",
                    ),
                )
                writer.write(
                    output_radar,
                    radar_record[1],
                    typestore.serialize_cdr(
                        radar_message,
                        "sensor_msgs/msg/PointCloud2",
                    ),
                )
    return len(images), minimum_points, maximum_points


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_dir", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--horizontal-flip", action="store_true")
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"refusing to overwrite {args.output}")
    args.output.mkdir(parents=True)

    report = [
        f"Source: {args.input_dir}",
        f"Camera topic: {OUTPUT_IMAGE_TOPIC}",
        f"Radar topic: {OUTPUT_RADAR_TOPIC}",
        "Fixed frame: radar",
        f"Horizontal image flip: {args.horizontal_flip}",
        "",
    ]
    bags = sorted(args.input_dir.glob("*.bag"))
    for source in bags:
        destination = args.output / source.stem
        count, minimum, maximum = convert_bag(
            source,
            destination,
            args.horizontal_flip,
        )
        line = (
            f"{source.name}: {count} synchronized frames, "
            f"radar points/frame={minimum}..{maximum}"
        )
        report.append(line)
        print(line)

    (args.output / "CONVERSION_REPORT.txt").write_text(
        "\n".join(report) + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
