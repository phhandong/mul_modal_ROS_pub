#!/usr/bin/env python3
"""Extract JPEG images and XYZI PCD files from ROS 1 or ROS 2 bags."""
import argparse
from pathlib import Path
import cv2
import numpy as np
from rosbags.highlevel import AnyReader
from pcd_io import pointcloud_xyzi, write_pcd

def stamp_ns(msg, fallback):
    header = getattr(msg, 'header', None); stamp = getattr(header, 'stamp', None)
    if stamp is None: return fallback
    return int(getattr(stamp, 'sec', 0)) * 1_000_000_000 + int(getattr(stamp, 'nanosec', getattr(stamp, 'nsec', 0)))

def unique(path):
    if not path.exists(): return path
    index = 1
    while True:
        candidate = path.with_stem(f'{path.stem}_{index}')
        if not candidate.exists(): return candidate
        index += 1

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path, help='ROS 1 .bag, rosbag2 directory, or MCAP file')
    parser.add_argument('--image-topic', action='append', default=[], help='Repeat for each compressed-image topic')
    parser.add_argument('--pointcloud-topic', default='/pointcloud')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ascii', action='store_true', help='Write ASCII instead of binary PCD')
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=True)
    image_topics = set(args.image_topic)
    with AnyReader([args.bag]) as reader:
        selected = [connection for connection in reader.connections if connection.topic in image_topics or connection.topic == args.pointcloud_topic]
        for connection, timestamp, raw in reader.messages(connections=selected):
            msg = reader.deserialize(raw, connection.msgtype); stamp = stamp_ns(msg, timestamp)
            if connection.topic in image_topics:
                encoded = np.frombuffer(msg.data, dtype=np.uint8); image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
                if image is None: continue
                folder = args.output / 'images' / connection.topic.strip('/').replace('/', '_'); folder.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(unique(folder / f'{stamp}.jpg')), image)
            elif connection.topic == args.pointcloud_topic:
                write_pcd(unique(args.output / 'pointcloud' / f'{stamp}.pcd'), pointcloud_xyzi(msg), binary=not args.ascii)

if __name__ == '__main__': main()
