#!/usr/bin/env python3
"""Horizontally mirror calibration images while keeping CameraInfo consistent."""

import argparse
import shutil
from pathlib import Path

import cv2
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore


IMAGE_TOPIC = "/sync/image_visible"
CAMERA_INFO_TOPIC = "/sync/camera_info"


def mirror_image_message(message):
    image = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("could not decode compressed image")
    mirrored = cv2.flip(image, 1)
    ok, encoded = cv2.imencode(
        ".jpg", mirrored, [cv2.IMWRITE_JPEG_QUALITY, 95]
    )
    if not ok:
        raise ValueError("could not encode mirrored image")
    message.data = encoded.reshape(-1).astype(np.uint8, copy=False)
    return message, image, mirrored


def mirror_camera_info(message):
    width = int(message.width)
    k = np.asarray(message.k, dtype=np.float64).copy()
    p = np.asarray(message.p, dtype=np.float64).copy()
    k[0] = -k[0]
    k[2] = (width - 1) - k[2]
    p[0] = -p[0]
    p[2] = (width - 1) - p[2]
    message.k = k
    message.p = p
    return message


def convert_bag(
    source_dir: Path,
    destination_dir: Path,
    preserve_camera_info: bool = False,
) -> tuple[int, int]:
    typestore = get_typestore(Stores.ROS2_JAZZY)
    image_count = info_count = 0
    with AnyReader([source_dir]) as reader:
        with Writer(
            destination_dir, version=9, storage_plugin=StoragePlugin.MCAP
        ) as writer:
            output_connections = {
                connection.id: writer.add_connection(
                    connection.topic,
                    connection.msgtype,
                    typestore=typestore,
                    serialization_format=connection.ext.serialization_format,
                    offered_qos_profiles=connection.ext.offered_qos_profiles,
                )
                for connection in reader.connections
            }
            for connection, timestamp, raw in reader.messages():
                message = reader.deserialize(raw, connection.msgtype)
                if connection.topic == IMAGE_TOPIC:
                    message, _, _ = mirror_image_message(message)
                    image_count += 1
                elif connection.topic == CAMERA_INFO_TOPIC:
                    if not preserve_camera_info:
                        message = mirror_camera_info(message)
                    info_count += 1
                writer.write(
                    output_connections[connection.id],
                    timestamp,
                    typestore.serialize_cdr(message, connection.msgtype),
                )
    return image_count, info_count


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_dir", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--preserve-camera-info",
        action="store_true",
        help="mirror images but retain CameraInfo K/P exactly as stored",
    )
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"refusing to overwrite {args.output}")
    args.output.mkdir(parents=True, exist_ok=True)

    for source_dir in sorted(
        (path for path in args.input_dir.iterdir() if path.is_dir()),
        key=lambda path: int(path.name),
    ):
        destination_dir = args.output / source_dir.name
        if destination_dir.exists():
            raise SystemExit(f"refusing to overwrite {destination_dir}")
        image_count, info_count = convert_bag(
            source_dir,
            destination_dir,
            preserve_camera_info=args.preserve_camera_info,
        )
        print(
            f"{source_dir.name}: mirrored {image_count} images, "
            f"{'preserved' if args.preserve_camera_info else 'updated'} "
            f"{info_count} CameraInfo messages"
        )

    for source_file in args.input_dir.iterdir():
        if source_file.is_file():
            shutil.copy2(source_file, args.output / source_file.name)

    camera_info_note = (
        "CameraInfo messages were retained byte-for-byte with their original "
        "positive focal lengths.\n"
        if args.preserve_camera_info
        else
        "CameraInfo K and P were updated exactly for the mirror operation:\n"
        "fx'=-fx, cx'=width-1-cx. Distortion coefficients are unchanged.\n"
    )
    (args.output / "MIRROR_NOTICE.txt").write_text(
        "All /sync/image_visible images were horizontally mirrored.\n"
        + camera_info_note,
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
