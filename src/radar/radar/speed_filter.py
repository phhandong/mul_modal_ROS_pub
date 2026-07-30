#!/usr/bin/env python3
"""Filter recorded ARS408 PointCloud2 messages by relative velocity."""

import math
import struct

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


class Ars408SpeedFilter(Node):
    def __init__(self) -> None:
        super().__init__("ars408_speed_filter")
        self.input_topic = self.declare_parameter(
            "input_topic", "/ars408/points_raw"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/ars408/points"
        ).value
        self.declare_parameter("min_speed_mps", 0.0)
        self.declare_parameter("max_speed_mps", -1.0)
        self.declare_parameter("speed_filter_mode", "magnitude")
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.status_publisher = self.create_publisher(
            String, "/ars408/filter_status", 10
        )
        self.subscription = self.create_subscription(
            PointCloud2, self.input_topic, self.filter_cloud, 10
        )
        self._reported_layout_error = False

    def filter_cloud(self, message: PointCloud2) -> None:
        fields = {field.name: field for field in message.fields}
        mode = str(self.get_parameter("speed_filter_mode").value)
        required = (
            ("vx",) if mode == "longitudinal"
            else ("vy",) if mode == "lateral"
            else ("speed",) if "speed" in fields
            else ("vx", "vy")
        )
        if any(
            name not in fields or fields[name].datatype != PointField.FLOAT32
            for name in required
        ):
            if not self._reported_layout_error:
                self.get_logger().error(
                    f"Input cloud lacks FLOAT32 velocity fields required for {mode}: {required}"
                )
                self._reported_layout_error = True
            return

        minimum = float(self.get_parameter("min_speed_mps").value)
        maximum = float(self.get_parameter("max_speed_mps").value)
        endian = ">" if message.is_bigendian else "<"
        kept = []
        source = bytes(message.data)
        for row in range(message.height):
            row_start = row * message.row_step
            for column in range(message.width):
                start = row_start + column * message.point_step
                if mode == "longitudinal":
                    speed = abs(struct.unpack_from(
                        endian + "f", source, start + fields["vx"].offset
                    )[0])
                elif mode == "lateral":
                    speed = abs(struct.unpack_from(
                        endian + "f", source, start + fields["vy"].offset
                    )[0])
                elif "speed" in fields:
                    speed = struct.unpack_from(
                        endian + "f", source, start + fields["speed"].offset
                    )[0]
                else:
                    vx = struct.unpack_from(
                        endian + "f", source, start + fields["vx"].offset
                    )[0]
                    vy = struct.unpack_from(
                        endian + "f", source, start + fields["vy"].offset
                    )[0]
                    speed = math.hypot(vx, vy)
                if speed >= minimum and (maximum < 0.0 or speed <= maximum):
                    kept.append(source[start:start + message.point_step])

        output = PointCloud2()
        output.header = message.header
        output.height = 1
        output.width = len(kept)
        output.fields = message.fields
        output.is_bigendian = message.is_bigendian
        output.point_step = message.point_step
        output.row_step = message.point_step * len(kept)
        output.data = b"".join(kept)
        output.is_dense = message.is_dense
        self.publisher.publish(output)
        self.status_publisher.publish(String(
            data=(
                f"ARS408 playback filter: points={len(kept)}/{message.width * message.height}, "
                f"mode={mode}, speed=[{minimum}, {maximum}] m/s"
            )
        ))


def main() -> None:
    rclpy.init()
    node = Ars408SpeedFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
