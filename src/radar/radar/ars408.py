#!/usr/bin/env python3
"""ROS 2 ARS408 object-list receiver.

The radar itself speaks CAN.  This node supports local SocketCAN, the observed
USR-CANET200 13-byte TCP stream, and an explicit human-readable gateway format
``<can-id>#<hex-payload>`` (for example ``60A#0300012000000000``).
"""

import re
import socket
import struct
import math
import statistics
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

from .ars408_protocol import (
    CLUSTER_GENERAL_ID,
    CLUSTER_STATUS_ID,
    OBJECT_GENERAL_ID,
    OBJECT_STATUS_ID,
    RadarObject,
    decode_cluster_general,
    decode_cluster_status,
    decode_object_general,
    decode_object_status,
)


ASCII_CAN_FRAME = re.compile(r"^\s*(?:0x)?([0-9a-fA-F]{1,3})#([0-9a-fA-F]{0,16})\s*$")


class Ars408(Node):
    def __init__(self) -> None:
        super().__init__("ars408_radar")
        self.source = self.declare_parameter("source", "socketcan").value
        self.can_interface = self.declare_parameter("can_interface", "can0").value
        self.gateway_ip = self.declare_parameter("gateway_ip", "192.168.2.119").value
        self.gateway_port = self.declare_parameter("gateway_port", 29536).value
        self.gateway_transport = self.declare_parameter(
            "gateway_transport", "usr_canet_tcp_server"
        ).value
        self.sensor_id = self.declare_parameter("sensor_id", 0).value
        self.frame_id = self.declare_parameter("frame_id", "ars408").value
        self.topic = self.declare_parameter("pointcloud_topic", "/ars408/points").value
        self.raw_topic = self.declare_parameter(
            "raw_pointcloud_topic", "/ars408/points_raw"
        ).value
        self.declare_parameter("min_speed_mps", 0.0)
        self.declare_parameter("max_speed_mps", -1.0)
        self.declare_parameter("speed_filter_mode", "magnitude")
        self.points_pub = self.create_publisher(PointCloud2, self.topic, 10)
        self.raw_points_pub = self.create_publisher(PointCloud2, self.raw_topic, 10)
        self.status_pub = self.create_publisher(String, "/ars408/status", 10)
        self.sock: Optional[socket.socket] = None
        self.objects: Dict[int, RadarObject] = {}
        self.expected_count: Optional[int] = None
        self.measurement_counter: Optional[int] = None
        self.received_bytes = 0
        self.decoded_can_frames = 0
        self.rejected_gateway_records = 0
        self._tcp_buffer = b""
        self._usr_clients: Dict[socket.socket, bytes] = {}
        self._reported_bad_record = False
        self.last_raw_count = 0
        self.last_filtered_count = 0
        self.last_speed_min = 0.0
        self.last_speed_median = 0.0
        self.last_speed_max = 0.0
        self._open_source()
        self.create_timer(0.005, self.poll)
        self.create_timer(1.0, self.report_status)

    def _open_source(self) -> None:
        if self.source == "socketcan":
            if not hasattr(socket, "AF_CAN"):
                raise RuntimeError("This platform does not support SocketCAN")
            self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.sock.bind((self.can_interface,))
        elif self.source == "gateway":
            if self.gateway_transport not in (
                "udp_ascii", "tcp_ascii", "usr_canet_tcp_server"
            ):
                raise RuntimeError(
                    "gateway_transport must be udp_ascii, tcp_ascii, or usr_canet_tcp_server"
                )
            if not 1 <= self.gateway_port <= 65535:
                raise RuntimeError(
                    "gateway_port is required: the ARS408 manuals do not specify the CAN-gateway port"
                )
            kind = socket.SOCK_DGRAM if self.gateway_transport == "udp_ascii" else socket.SOCK_STREAM
            self.sock = socket.socket(socket.AF_INET, kind)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            if self.gateway_transport == "usr_canet_tcp_server":
                self.sock.bind(("0.0.0.0", self.gateway_port))
                self.sock.listen()
            elif kind == socket.SOCK_DGRAM:
                # A UDP gateway pushes frames to the configured local port.  Its
                # source address is verified in poll() before decoding a frame.
                self.sock.bind(("0.0.0.0", self.gateway_port))
            else:
                self.sock.settimeout(3.0)
                self.sock.connect((self.gateway_ip, self.gateway_port))
            self.sock.setblocking(False)
        else:
            raise RuntimeError("source must be socketcan or gateway")
        self.get_logger().info(f"ARS408 source={self.source}")

    def poll(self) -> None:
        assert self.sock is not None
        if self.source == "gateway" and self.gateway_transport == "usr_canet_tcp_server":
            self._poll_usr_canet()
            return
        try:
            while True:
                if self.source == "gateway" and self.gateway_transport == "udp_ascii":
                    packet, peer = self.sock.recvfrom(4096)
                    if peer[0] != self.gateway_ip:
                        self.get_logger().warning(f"Ignored UDP CAN data from {peer[0]}")
                        continue
                else:
                    packet = self.sock.recv(4096)
                if not packet:
                    return
                self.received_bytes += len(packet)
                if self.source == "socketcan":
                    if len(packet) < 16:
                        continue
                    can_id, dlc, raw = struct.unpack("=IB3x8s", packet[:16])
                    self.decoded_can_frames += 1
                    self.handle_frame(can_id & 0x1FFFFFFF, raw[:dlc])
                else:
                    self._parse_gateway_packet(packet)
        except BlockingIOError:
            pass
        except OSError as exc:
            self.get_logger().warning(f"CAN input error: {exc}")

    def _poll_usr_canet(self) -> None:
        assert self.sock is not None
        try:
            while True:
                client, peer = self.sock.accept()
                client.setblocking(False)
                if peer[0] != self.gateway_ip:
                    self.get_logger().warning(f"Rejected USR-CANET client from {peer[0]}")
                    client.close()
                    continue
                self._usr_clients[client] = b""
                self.get_logger().info(f"USR-CANET client connected from {peer[0]}:{peer[1]}")
        except BlockingIOError:
            pass

        for client in list(self._usr_clients):
            try:
                while True:
                    packet = client.recv(65536)
                    if not packet:
                        self._close_usr_client(client)
                        break
                    self.received_bytes += len(packet)
                    buffer = self._usr_clients[client] + packet
                    while len(buffer) >= 13:
                        record, buffer = buffer[:13], buffer[13:]
                        dlc = record[0]
                        can_id = int.from_bytes(record[1:5], "big") & 0x1FFFFFFF
                        if dlc > 8:
                            self._reject_gateway_record(record)
                            continue
                        self.decoded_can_frames += 1
                        self.handle_frame(can_id, record[5:5 + dlc])
                    if client in self._usr_clients:
                        self._usr_clients[client] = buffer
            except BlockingIOError:
                pass
            except OSError as exc:
                self.get_logger().warning(f"USR-CANET input error: {exc}")
                self._close_usr_client(client)

    def _close_usr_client(self, client: socket.socket) -> None:
        self._usr_clients.pop(client, None)
        try:
            client.close()
        except OSError:
            pass

    def handle_frame(self, can_id: int, payload: bytes) -> None:
        offset = int(self.sensor_id) * 0x10
        if can_id in (OBJECT_STATUS_ID + offset, CLUSTER_STATUS_ID + offset):
            is_cluster = can_id == CLUSTER_STATUS_ID + offset
            status = (
                decode_cluster_status(payload)
                if is_cluster else decode_object_status(payload)
            )
            if status is None:
                return
            if self.expected_count is not None:
                self.publish_cycle()
            self.objects = {}
            self.expected_count = status.count
            self.measurement_counter = status.measurement_counter
            if status.count == 0:
                self.publish_cycle()
        elif can_id in (
            OBJECT_GENERAL_ID + offset, CLUSTER_GENERAL_ID + offset
        ) and self.expected_count is not None:
            target = (
                decode_cluster_general(payload)
                if can_id == CLUSTER_GENERAL_ID + offset
                else decode_object_general(payload)
            )
            if target is not None:
                self.objects[target.object_id] = target
                if len(self.objects) >= self.expected_count:
                    self.publish_cycle()

    def publish_cycle(self) -> None:
        if self.expected_count is None:
            return
        points = sorted(self.objects.values(), key=lambda item: item.object_id)
        stamp = self.get_clock().now().to_msg()
        self.raw_points_pub.publish(self._pointcloud(points, stamp))

        minimum = float(self.get_parameter("min_speed_mps").value)
        maximum = float(self.get_parameter("max_speed_mps").value)
        mode = str(self.get_parameter("speed_filter_mode").value)
        filtered = [
            point for point in points
            if self._speed_allowed(point, mode, minimum, maximum)
        ]
        speeds = [math.hypot(point.vx_mps, point.vy_mps) for point in points]
        self.points_pub.publish(self._pointcloud(filtered, stamp))
        self.last_raw_count = len(points)
        self.last_filtered_count = len(filtered)
        if speeds:
            self.last_speed_min = min(speeds)
            self.last_speed_median = statistics.median(speeds)
            self.last_speed_max = max(speeds)
        else:
            self.last_speed_min = 0.0
            self.last_speed_median = 0.0
            self.last_speed_max = 0.0
        self.expected_count = None

    def _pointcloud(self, points: list[RadarObject], stamp) -> PointCloud2:
        payload = b"".join(
            struct.pack(
                "<fffffffHBx",
                point.x_m,
                point.y_m,
                0.0,
                point.rcs_dbm2,
                point.vx_mps,
                point.vy_mps,
                math.hypot(point.vx_mps, point.vy_mps),
                point.object_id,
                point.dynamic_property,
            )
            for point in points
        )
        message = PointCloud2()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.height = 1
        message.width = len(points)
        message.fields = [
            PointField(name=name, offset=offset, datatype=datatype, count=1)
            for name, offset, datatype in (
                ("x", 0, PointField.FLOAT32),
                ("y", 4, PointField.FLOAT32),
                ("z", 8, PointField.FLOAT32),
                ("intensity", 12, PointField.FLOAT32),
                ("vx", 16, PointField.FLOAT32),
                ("vy", 20, PointField.FLOAT32),
                ("speed", 24, PointField.FLOAT32),
                ("target_id", 28, PointField.UINT16),
                ("dynamic_property", 30, PointField.UINT8),
            )
        ]
        message.is_bigendian = False
        message.point_step = 32
        message.row_step = 32 * len(points)
        message.data = payload
        message.is_dense = True
        return message

    def _speed_allowed(
        self, point: RadarObject, mode: str, minimum: float, maximum: float
    ) -> bool:
        if mode == "longitudinal":
            speed = abs(point.vx_mps)
        elif mode == "lateral":
            speed = abs(point.vy_mps)
        else:
            speed = math.hypot(point.vx_mps, point.vy_mps)
        return speed >= minimum and (maximum < 0.0 or speed <= maximum)

    def _parse_gateway_packet(self, packet: bytes) -> None:
        if self.gateway_transport == "tcp_ascii":
            self._tcp_buffer += packet
            records = self._tcp_buffer.split(b"\n")
            self._tcp_buffer = records.pop()
            if len(self._tcp_buffer) > 8192:
                self._reject_gateway_record(self._tcp_buffer)
                self._tcp_buffer = b""
        else:
            records = packet.splitlines()
        for raw_record in records:
            record = raw_record.rstrip(b"\r").decode("ascii", errors="replace")
            match = ASCII_CAN_FRAME.match(record)
            if not match:
                if raw_record:
                    self._reject_gateway_record(raw_record)
                continue
            self.decoded_can_frames += 1
            self.handle_frame(int(match.group(1), 16), bytes.fromhex(match.group(2)))

    def _reject_gateway_record(self, record: bytes) -> None:
        self.rejected_gateway_records += 1
        if not self._reported_bad_record:
            sample = record[:64].hex(" ")
            self.get_logger().warning(
                f"Gateway data does not match CAN_ID#HEX format; first bytes: {sample}"
            )
            self._reported_bad_record = True

    def report_status(self) -> None:
        cycle = "none" if self.measurement_counter is None else str(self.measurement_counter)
        self.status_pub.publish(String(
            data=(
                f"ARS408: source={self.source}, rx_bytes={self.received_bytes}, "
                f"can_frames={self.decoded_can_frames}, "
                f"rejected_records={self.rejected_gateway_records}, "
                f"points={self.last_filtered_count}/{self.last_raw_count}, "
                f"speed_mps[min/median/max]="
                f"{self.last_speed_min:.2f}/{self.last_speed_median:.2f}/"
                f"{self.last_speed_max:.2f}, "
                f"measurement_counter={cycle}"
            )
        ))


def main() -> None:
    rclpy.init()
    node: Optional[Ars408] = None
    try:
        node = Ars408()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
