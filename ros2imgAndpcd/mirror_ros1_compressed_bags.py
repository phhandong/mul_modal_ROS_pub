#!/usr/bin/env python3
"""Copy ROS1 bags while horizontally flipping one CompressedImage topic."""

import argparse
from pathlib import Path

import cv2
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag1 import Writer


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_dir", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--image-topic", default="/sync/image_visible")
    args = parser.parse_args()

    bags = sorted(args.input_dir.glob("*.bag"), key=lambda path: int(path.stem))
    if not bags:
        raise SystemExit(f"no ROS1 bags in {args.input_dir}")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    for source in bags:
        target = args.output_dir / source.name
        if target.exists():
            raise SystemExit(f"refusing to overwrite {target}")
        with AnyReader([source]) as reader, Writer(target) as writer:
            output_connections = {}
            for connection in reader.connections:
                output_connections[connection.id] = writer.add_connection(
                    connection.topic,
                    connection.msgtype,
                    msgdef=connection.msgdef.data,
                    md5sum=connection.digest,
                    callerid=connection.ext.callerid,
                    latching=connection.ext.latching,
                )

            flipped = 0
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic != args.image_topic:
                    writer.write(output_connections[connection.id], timestamp, rawdata)
                    continue

                message = reader.deserialize(rawdata, connection.msgtype)
                image = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError(f"{source}: cannot decode image")
                suffix = ".png" if "png" in message.format.lower() else ".jpg"
                success, encoded = cv2.imencode(suffix, cv2.flip(image, 1))
                if not success:
                    raise ValueError(f"{source}: cannot encode image")
                message.data = encoded
                writer.write(
                    output_connections[connection.id],
                    timestamp,
                    reader.typestore.serialize_ros1(message, connection.msgtype),
                )
                flipped += 1
        print(f"{source.name}: flipped {flipped} images")


if __name__ == "__main__":
    main()
