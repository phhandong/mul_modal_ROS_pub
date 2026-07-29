#!/usr/bin/env python3
"""Project synchronized LiDAR clouds onto raw camera images."""

import argparse
import re
from pathlib import Path

import cv2
import numpy as np
import yaml
from rosbags.highlevel import AnyReader

from pcd_io import pointcloud_xyzi


IMAGE_TOPIC = "/sync/image_visible"
POINTCLOUD_TOPIC = "/sync/pointcloud"


def discover_bags(input_dir: Path) -> list[Path]:
    """Return ROS1 bag files or ROS2 bag directories in numeric order."""
    sources = list(input_dir.glob("*.bag"))
    sources.extend(
        path
        for path in input_dir.iterdir()
        if path.is_dir() and (path / "metadata.yaml").is_file()
    )
    return sorted(
        sources,
        key=lambda path: int(path.stem if path.is_file() else path.name),
    )


def load_urdf_pose_matrix(path: Path) -> np.ndarray:
    text = path.read_text(encoding="utf-8")
    match = re.search(r"Matrix:\s*\n((?:[^\n]*\n){4})", text)
    if not match:
        raise ValueError(f"could not find 4x4 Matrix in {path}")
    values = [
        [float(value) for value in line.split()]
        for line in match.group(1).strip().splitlines()
    ]
    matrix = np.asarray(values, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError(f"expected 4x4 transform, got {matrix.shape}")
    return matrix


def load_camera_info(path: Path):
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    matrix = np.asarray(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(
        data["distortion_coefficients"]["data"], dtype=np.float64
    )
    return matrix, distortion, int(data["image_width"]), int(data["image_height"])


def project_overlay(
    image: np.ndarray,
    lidar: np.ndarray,
    lidar_to_camera: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> tuple[np.ndarray, int]:
    camera_points = (
        lidar[:, :3] @ lidar_to_camera[:3, :3].T + lidar_to_camera[:3, 3]
    )
    front = np.isfinite(camera_points).all(axis=1) & (camera_points[:, 2] > 0.1)
    camera_points = camera_points[front]
    if not len(camera_points):
        return image.copy(), 0

    pixels, _ = cv2.projectPoints(
        camera_points,
        np.zeros(3),
        np.zeros(3),
        camera_matrix,
        distortion,
    )
    pixels = pixels.reshape(-1, 2)
    height, width = image.shape[:2]
    inside = (
        (pixels[:, 0] >= 0)
        & (pixels[:, 0] < width)
        & (pixels[:, 1] >= 0)
        & (pixels[:, 1] < height)
    )
    pixels = pixels[inside]
    depths = camera_points[inside, 2]
    if not len(pixels):
        return image.copy(), 0

    # Draw far points first so nearer returns remain visible.
    order = np.argsort(depths)[::-1]
    pixels = pixels[order].astype(np.int32)
    depths = depths[order]
    lo, hi = np.percentile(depths, [2, 98])
    values = np.clip((depths - lo) / max(hi - lo, 1e-6) * 255, 0, 255).astype(
        np.uint8
    )
    # Reverse TURBO so red/yellow indicates near and blue indicates far.
    colors = cv2.applyColorMap((255 - values).reshape(-1, 1), cv2.COLORMAP_TURBO)[
        :, 0
    ]

    overlay = image.copy()
    for (x_pixel, y_pixel), color in zip(pixels, colors):
        cv2.circle(
            overlay,
            (int(x_pixel), int(y_pixel)),
            2,
            tuple(int(value) for value in color),
            -1,
            cv2.LINE_AA,
        )
    return overlay, len(pixels)


def process_bag(
    source: Path,
    output_dir: Path,
    preview_path: Path,
    transform: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    expected_size: tuple[int, int],
    horizontal_flip: bool = False,
    save_frames: bool = True,
) -> tuple[int, float, int, int]:
    if save_frames:
        output_dir.mkdir(parents=True, exist_ok=True)
    preview_path.parent.mkdir(parents=True, exist_ok=True)
    total_projected = 0
    minimum_projected = 2**63 - 1

    with AnyReader([source]) as reader:
        image_connection = next(
            item for item in reader.connections if item.topic == IMAGE_TOPIC
        )
        cloud_connection = next(
            item for item in reader.connections if item.topic == POINTCLOUD_TOPIC
        )
        image_messages = list(reader.messages(connections=[image_connection]))
        cloud_messages = list(reader.messages(connections=[cloud_connection]))
        count = min(len(image_messages), len(cloud_messages))
        duration = max((image_messages[-1][1] - image_messages[0][1]) / 1e9, 0.1)
        fps = float(np.clip((count - 1) / duration, 1.0, 15.0))

        writer = cv2.VideoWriter(
            str(preview_path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            fps,
            expected_size,
        )
        if not writer.isOpened():
            raise RuntimeError(f"could not create {preview_path}")
        try:
            for index in range(count):
                image_message = reader.deserialize(
                    image_messages[index][2], image_connection.msgtype
                )
                cloud_message = reader.deserialize(
                    cloud_messages[index][2], cloud_connection.msgtype
                )
                image = cv2.imdecode(
                    np.frombuffer(image_message.data, dtype=np.uint8),
                    cv2.IMREAD_COLOR,
                )
                if image is None:
                    raise ValueError(f"{source.name}: image {index} could not be decoded")
                if (image.shape[1], image.shape[0]) != expected_size:
                    raise ValueError(
                        f"{source.name}: image is {image.shape[1]}x{image.shape[0]}, "
                        f"expected {expected_size[0]}x{expected_size[1]}"
                    )
                if horizontal_flip:
                    image = cv2.flip(image, 1)
                overlay, projected = project_overlay(
                    image,
                    pointcloud_xyzi(cloud_message),
                    transform,
                    camera_matrix,
                    distortion,
                )
                cv2.putText(
                    overlay,
                    f"{source.name}  frame {index + 1}/{count}  projected: {projected:,}",
                    (18, image.shape[0] - 22),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                if save_frames:
                    timestamp = image_messages[index][1]
                    name = f"{timestamp / 1e9:.6f}".replace(".", "_")
                    cv2.imwrite(
                        str(output_dir / f"{name}.jpg"),
                        overlay,
                        [cv2.IMWRITE_JPEG_QUALITY, 92],
                    )
                writer.write(overlay)
                total_projected += projected
                minimum_projected = min(minimum_projected, projected)
        finally:
            writer.release()
    return count, fps, minimum_projected, total_projected


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_dir", type=Path)
    parser.add_argument("--extrinsic", type=Path, required=True)
    parser.add_argument("--camera-info", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--previews", type=Path, required=True)
    parser.add_argument(
        "--horizontal-flip",
        action="store_true",
        help="flip each source image horizontally before projecting",
    )
    parser.add_argument(
        "--video-only",
        action="store_true",
        help="write preview videos and report without individual JPEG frames",
    )
    args = parser.parse_args()

    # lidar2cam.txt stores a URDF parent->child pose (camera pose expressed in
    # the LiDAR frame). Coordinate conversion from LiDAR points into the camera
    # frame therefore uses its inverse.
    urdf_pose = load_urdf_pose_matrix(args.extrinsic)
    lidar_to_camera = np.linalg.inv(urdf_pose)
    camera_matrix, distortion, width, height = load_camera_info(args.camera_info)

    report = [
        f"Extrinsic source: {args.extrinsic}",
        "Transform used for point coordinates: inverse(Matrix), because Matrix is "
        "the URDF parent->child pose.",
        "LiDAR-to-camera coordinate transform:",
        np.array2string(lidar_to_camera, precision=10, suppress_small=False),
        "",
    ]
    bags = discover_bags(args.input_dir)
    if not bags:
        raise SystemExit(f"no ROS1/ROS2 bags found in {args.input_dir}")
    for bag in bags:
        bag_name = bag.stem if bag.is_file() else bag.name
        count, fps, minimum, total = process_bag(
            bag,
            args.output / bag_name,
            args.previews / f"{bag_name}_projected_preview.mp4",
            lidar_to_camera,
            camera_matrix,
            distortion,
            (width, height),
            horizontal_flip=args.horizontal_flip,
            save_frames=not args.video_only,
        )
        line = (
            f"{bag.name}: {count} frames, {fps:.2f} fps, "
            f"projected points/frame min={minimum}, mean={total / count:.1f}"
        )
        report.append(line)
        print(line)

    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "PROJECTION_REPORT.txt").write_text(
        "\n".join(report) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    main()
