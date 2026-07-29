#!/usr/bin/env python3
"""Convert synchronized ROS 1 camera/LiDAR bags into small ROS 2 MCAP bags."""

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore


IMAGE_TOPIC = "/sync/image_visible"
POINTCLOUD_TOPIC = "/sync/pointcloud"
CAMERA_INFO_TOPIC = "/sync/camera_info"


def uniform_indices(count: int, samples: int) -> list[int]:
    if count < samples:
        raise ValueError(f"bag contains only {count} synchronized frames, need {samples}")
    return np.linspace(0, count - 1, samples, dtype=int).tolist()


def image_size(reader: AnyReader, connection, raw: bytes) -> tuple[int, int]:
    message = reader.deserialize(raw, connection.msgtype)
    image = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError("failed to decode the first compressed image")
    return image.shape[1], image.shape[0]


def load_calibration(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    return {
        "width": int(data["image_width"]),
        "height": int(data["image_height"]),
        "model": str(data["distortion_model"]),
        "d": np.asarray(data["distortion_coefficients"]["data"], dtype=np.float64),
        "k": np.asarray(data["camera_matrix"]["data"], dtype=np.float64),
        "r": np.asarray(data["rectification_matrix"]["data"], dtype=np.float64),
        "p": np.asarray(data["projection_matrix"]["data"], dtype=np.float64),
    }


def camera_info(
    typestore,
    stamp_ns: int,
    width: int,
    height: int,
    frame_id: str,
    calibration: dict | None,
):
    """Create CameraInfo from measured calibration, or a pinhole fallback."""
    time_cls = typestore.types["builtin_interfaces/msg/Time"]
    header_cls = typestore.types["std_msgs/msg/Header"]
    roi_cls = typestore.types["sensor_msgs/msg/RegionOfInterest"]
    info_cls = typestore.types["sensor_msgs/msg/CameraInfo"]

    if calibration:
        if (width, height) != (calibration["width"], calibration["height"]):
            raise ValueError(
                f"image is {width}x{height}, calibration is "
                f"{calibration['width']}x{calibration['height']}"
            )
        model = calibration["model"]
        d, k, r, p = (
            calibration["d"],
            calibration["k"],
            calibration["r"],
            calibration["p"],
        )
    else:
        focal = float(max(width, height))
        model = "plumb_bob"
        d = np.zeros(5, dtype=np.float64)
        k = np.array(
            [focal, 0.0, width / 2.0, 0.0, focal, height / 2.0, 0.0, 0.0, 1.0],
            dtype=np.float64,
        )
        r = np.eye(3, dtype=np.float64).reshape(-1)
        p = np.array(
            [
                focal,
                0.0,
                width / 2.0,
                0.0,
                0.0,
                focal,
                height / 2.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
            ],
            dtype=np.float64,
        )
    return info_cls(
        header=header_cls(
            stamp=time_cls(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000),
            frame_id=frame_id,
        ),
        height=height,
        width=width,
        distortion_model=model,
        d=d,
        k=k,
        r=r,
        p=p,
        binning_x=0,
        binning_y=0,
        roi=roi_cls(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False),
    )


def convert(
    source: Path,
    destination: Path,
    samples: int,
    calibration: dict | None,
) -> list[int]:
    typestore = get_typestore(Stores.ROS2_JAZZY)
    with AnyReader([source]) as reader:
        image_conn = next((c for c in reader.connections if c.topic == IMAGE_TOPIC), None)
        cloud_conn = next((c for c in reader.connections if c.topic == POINTCLOUD_TOPIC), None)
        if image_conn is None or cloud_conn is None:
            raise ValueError(f"{source.name}: required image or point-cloud topic is missing")

        images = list(reader.messages(connections=[image_conn]))
        clouds = list(reader.messages(connections=[cloud_conn]))
        pair_count = min(len(images), len(clouds))
        selected = uniform_indices(pair_count, samples)
        width, height = image_size(reader, image_conn, images[0][2])

        with Writer(destination, version=9, storage_plugin=StoragePlugin.MCAP) as writer:
            out_image = writer.add_connection(IMAGE_TOPIC, image_conn.msgtype, typestore=typestore)
            out_cloud = writer.add_connection(POINTCLOUD_TOPIC, cloud_conn.msgtype, typestore=typestore)
            out_info = writer.add_connection(
                CAMERA_INFO_TOPIC, "sensor_msgs/msg/CameraInfo", typestore=typestore
            )

            for index in selected:
                _, image_time, image_raw = images[index]
                _, cloud_time, cloud_raw = clouds[index]
                image_msg = reader.deserialize(image_raw, image_conn.msgtype)
                cloud_msg = reader.deserialize(cloud_raw, cloud_conn.msgtype)
                frame_id = image_msg.header.frame_id or "camera"
                info_msg = camera_info(
                    typestore, image_time, width, height, frame_id, calibration
                )

                writer.write(
                    out_image,
                    image_time,
                    typestore.serialize_cdr(image_msg, image_conn.msgtype),
                )
                writer.write(
                    out_info,
                    image_time,
                    typestore.serialize_cdr(info_msg, "sensor_msgs/msg/CameraInfo"),
                )
                writer.write(
                    out_cloud,
                    cloud_time,
                    typestore.serialize_cdr(cloud_msg, cloud_conn.msgtype),
                )
    return selected


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_dir", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=5)
    parser.add_argument("--camera-info-yaml", type=Path)
    args = parser.parse_args()
    calibration = (
        load_calibration(args.camera_info_yaml) if args.camera_info_yaml else None
    )

    bags = sorted(args.input_dir.glob("*.bag"), key=lambda path: int(path.stem))
    if not bags:
        raise SystemExit(f"no .bag files found in {args.input_dir}")
    args.output.mkdir(parents=True, exist_ok=True)

    for source in bags:
        destination = args.output / source.stem
        if destination.exists():
            raise SystemExit(f"refusing to overwrite existing output: {destination}")
        selected = convert(source, destination, args.frames, calibration)
        print(f"{source.name} -> {destination} (source frame indices: {selected})")

    notice = args.output / "CAMERA_INFO_NOTICE.txt"
    if args.camera_info_yaml:
        notice.write_text(
            "The source bags do not contain sensor_msgs/CameraInfo.\n"
            f"Measured CameraInfo was injected from: {args.camera_info_yaml}\n",
            encoding="utf-8",
        )
    else:
        notice.write_text(
            "The source bags do not contain sensor_msgs/CameraInfo.\n"
            "Each generated MCAP therefore contains a placeholder pinhole CameraInfo.\n",
            encoding="utf-8",
        )


if __name__ == "__main__":
    main()
