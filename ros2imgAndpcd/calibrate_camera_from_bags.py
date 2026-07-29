#!/usr/bin/env python3
"""Select one checkerboard image per readable bag and calibrate the camera."""

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


IMAGE_TOPIC = "/sync/image_visible"
PATTERN_SIZE = (8, 7)
SQUARE_SIZE_M = 0.02
DETECTION_FLAGS = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
SUBPIX_CRITERIA = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
    50,
    0.001,
)
# The board occupies a limited central image region, so higher radial terms
# are not observable reliably (the unconstrained fit drives k3 above 100).
# A single radial coefficient is substantially more stable under leave-one-out
# validation while increasing RMS by only about 0.005 px.
CALIBRATION_FLAGS = (
    cv2.CALIB_FIX_K2
    | cv2.CALIB_FIX_K3
    | cv2.CALIB_ZERO_TANGENT_DIST
)


def iter_normal_images(path: Path):
    with AnyReader([path]) as reader:
        connection = next(
            (item for item in reader.connections if item.topic == IMAGE_TOPIC), None
        )
        if connection is None:
            return
        for _, timestamp, raw in reader.messages(connections=[connection]):
            yield timestamp, reader.deserialize(raw, connection.msgtype)


def iter_salvaged_images(path: Path):
    """Read complete image records from the valid prefix of an unindexed ROS1 bag."""
    typestore = get_typestore(Stores.ROS1_NOETIC)
    connections = {}
    with path.open("rb") as source:
        if source.readline() != b"#ROSBAG V2.0\n":
            return
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
                            connection = connections.get(header.get_uint32("conn"))
                            if connection and connection[0] == IMAGE_TOPIC:
                                yield (
                                    header.get_time("time"),
                                    typestore.deserialize_ros1(
                                        raw_message, connection[1]
                                    ),
                                )
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
                break


def decode_image(message) -> np.ndarray | None:
    return cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)


def detect_corners(image: np.ndarray):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    preview = cv2.resize(gray, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    found, corners = cv2.findChessboardCorners(preview, PATTERN_SIZE, DETECTION_FLAGS)
    if not found:
        return None
    corners = corners * 2.0
    return cv2.cornerSubPix(gray, corners, (7, 7), (-1, -1), SUBPIX_CRITERIA)


def selection_score(image: np.ndarray, corners: np.ndarray) -> tuple[float, float, float]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    sharpness = cv2.Laplacian(gray, cv2.CV_64F).var()
    hull_area = cv2.contourArea(cv2.convexHull(corners.reshape(-1, 2)))
    image_area = image.shape[0] * image.shape[1]
    grid = np.mgrid[0 : PATTERN_SIZE[0], 0 : PATTERN_SIZE[1]].T.reshape(-1, 2)
    homography, _ = cv2.findHomography(
        grid.astype(np.float32), corners.reshape(-1, 2), method=0
    )
    projected = cv2.perspectiveTransform(
        grid.astype(np.float32).reshape(-1, 1, 2), homography
    )
    homography_rms = float(
        np.sqrt(np.mean(np.sum((corners - projected) ** 2, axis=2)))
    )
    # Corner-grid consistency is the primary selection criterion. Coverage and
    # sharpness only break ties; maximizing sharpness alone selected reflected
    # or locally misdetected corners in some bags.
    return (-homography_rms, hull_area / image_area, float(np.log1p(sharpness)))


def select_one_image(path: Path, recover: bool):
    iterator = iter_salvaged_images(path) if recover else iter_normal_images(path)
    best = None
    best_any = None
    decoded_count = detected_count = 0
    for timestamp, message in iterator:
        image = decode_image(message)
        if image is None:
            continue
        decoded_count += 1
        sharpness = cv2.Laplacian(
            cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), cv2.CV_64F
        ).var()
        if best_any is None or sharpness > best_any[0]:
            best_any = (sharpness, timestamp, image)
        corners = detect_corners(image)
        if corners is None:
            continue
        detected_count += 1
        score = selection_score(image, corners)
        if best is None or score > best[0]:
            best = (score, timestamp, image, corners)
    return best, best_any, decoded_count, detected_count


def object_points() -> np.ndarray:
    points = np.zeros((PATTERN_SIZE[0] * PATTERN_SIZE[1], 3), dtype=np.float32)
    points[:, :2] = (
        np.mgrid[0 : PATTERN_SIZE[0], 0 : PATTERN_SIZE[1]]
        .T.reshape(-1, 2)
        .astype(np.float32)
        * SQUARE_SIZE_M
    )
    return points


def calibrate(selected):
    image_size = (selected[0]["image"].shape[1], selected[0]["image"].shape[0])
    object_sets = [object_points() for _ in selected]
    image_sets = [item["corners"] for item in selected]
    rms, matrix, distortion, rotations, translations = cv2.calibrateCamera(
        object_sets,
        image_sets,
        image_size,
        None,
        None,
        flags=CALIBRATION_FLAGS,
    )

    per_view_rms = []
    for obj, observed, rotation, translation in zip(
        object_sets, image_sets, rotations, translations
    ):
        projected, _ = cv2.projectPoints(
            obj, rotation, translation, matrix, distortion
        )
        residuals = observed.reshape(-1, 2) - projected.reshape(-1, 2)
        per_view_rms.append(float(np.sqrt(np.mean(np.sum(residuals**2, axis=1)))))
    return rms, matrix, distortion.reshape(-1), per_view_rms


def write_camera_info(path: Path, matrix: np.ndarray, distortion: np.ndarray) -> None:
    projection = np.array(
        [
            matrix[0, 0],
            0.0,
            matrix[0, 2],
            0.0,
            0.0,
            matrix[1, 1],
            matrix[1, 2],
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
    )
    data = {
        "image_width": 1920,
        "image_height": 1080,
        "camera_name": "hik_camera",
        "camera_matrix": {"rows": 3, "cols": 3, "data": matrix.reshape(-1).tolist()},
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {
            "rows": 1,
            "cols": 5,
            "data": distortion[:5].tolist(),
        },
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": np.eye(3).reshape(-1).tolist(),
        },
        "projection_matrix": {"rows": 3, "cols": 4, "data": projection.tolist()},
    }
    with path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, sort_keys=False, allow_unicode=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("data_root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    image_dir = args.output / "selected_images"
    all_frames_dir = args.output / "one_frame_per_readable_bag"
    detection_dir = args.output / "detected_corners"
    image_dir.mkdir(exist_ok=True)
    all_frames_dir.mkdir(exist_ok=True)
    detection_dir.mkdir(exist_ok=True)

    selected = []
    extraction_report = []
    bags = sorted(
        args.data_root.glob("*/*.bag"),
        key=lambda path: (path.parent.name, int(path.stem)),
    )
    for bag in bags:
        with bag.open("rb") as source:
            magic = source.read(13)
        if magic != b"#ROSBAG V2.0\n":
            extraction_report.append(f"{bag.parent.name}/{bag.name}: skipped, invalid magic")
            print(extraction_report[-1])
            continue
        recover = bag.parent.name == "lidar_radar" and bag.stem == "21"
        try:
            best, best_any, decoded_count, detected_count = select_one_image(bag, recover)
        except Exception as error:
            extraction_report.append(
                f"{bag.parent.name}/{bag.name}: skipped, read error: {error}"
            )
            print(extraction_report[-1])
            continue
        name = f"{bag.parent.name}_{bag.stem}"
        if best_any is not None:
            cv2.imwrite(str(all_frames_dir / f"{name}.jpg"), best_any[2])
        if best is None:
            extraction_report.append(
                f"{bag.parent.name}/{bag.name}: no 8x7 checkerboard "
                f"({decoded_count} images checked)"
            )
            print(extraction_report[-1])
            continue

        score, timestamp, image, corners = best
        image_path = image_dir / f"{name}.png"
        detected_path = detection_dir / f"{name}.jpg"
        cv2.imwrite(str(image_path), image)
        drawn = image.copy()
        cv2.drawChessboardCorners(drawn, PATTERN_SIZE, corners, True)
        cv2.imwrite(str(detected_path), drawn, [cv2.IMWRITE_JPEG_QUALITY, 92])
        selected.append(
            {
                "name": name,
                "source": str(bag),
                "timestamp_ns": timestamp,
                "image": image,
                "corners": corners,
            }
        )
        extraction_report.append(
            f"{bag.parent.name}/{bag.name}: selected 1 of {detected_count} detected "
            f"frames ({decoded_count} checked)"
        )
        print(extraction_report[-1])

    if len(selected) < 5:
        raise SystemExit(f"only {len(selected)} usable calibration views found")

    rms, matrix, distortion, per_view_rms = calibrate(selected)
    write_camera_info(args.output / "camera_info.yaml", matrix, distortion)
    np.savez(
        args.output / "camera_calibration.npz",
        camera_matrix=matrix,
        distortion_coefficients=distortion,
        rms=rms,
    )

    report_lines = [
        "Camera calibration report",
        "=========================",
        f"Pattern: checkerboard {PATTERN_SIZE[0]}x{PATTERN_SIZE[1]} inner corners",
        f"Square spacing used: {SQUARE_SIZE_M:.6f} m",
        f"Image size: 1920x1080",
        f"Selected views: {len(selected)}",
        "Distortion fit: k1 estimated; k2, p1, p2, k3 fixed to zero",
        f"OpenCV global RMS reprojection error: {rms:.6f} px",
        "",
        "Camera matrix K:",
        np.array2string(matrix, precision=10, suppress_small=False),
        "",
        "Distortion [k1, k2, p1, p2, k3]:",
        np.array2string(distortion[:5], precision=10, suppress_small=False),
        "",
        "Per-view RMS:",
    ]
    report_lines.extend(
        f"  {item['name']}: {error:.6f} px"
        for item, error in zip(selected, per_view_rms)
    )
    report_lines.extend(["", "Extraction:", *extraction_report])
    (args.output / "calibration_report.txt").write_text(
        "\n".join(report_lines) + "\n", encoding="utf-8"
    )
    print(f"CALIBRATION_RMS={rms:.6f}")
    print(f"K={matrix.reshape(-1).tolist()}")
    print(f"D={distortion[:5].tolist()}")


if __name__ == "__main__":
    main()
