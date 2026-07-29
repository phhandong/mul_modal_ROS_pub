import socket
import time

import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Sentence
from message_interface.msg import AIS, RadarTarget


class UdpNode(Node):
    def __init__(self, name: str):
        super().__init__(name)
        bind_ip = self.declare_parameter('bind_ip', '0.0.0.0').value
        port = self.declare_parameter('port', 0).value
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind_ip, port))
        self.sock.setblocking(False)
        self.timer = self.create_timer(0.01, self.read_pending)

    def read_pending(self):
        while rclpy.ok():
            try:
                payload, _ = self.sock.recvfrom(8192)
            except BlockingIOError:
                return
            try:
                self.handle(payload.decode('ascii').strip())
            except (UnicodeDecodeError, ValueError, IndexError) as exc:
                self.get_logger().warning(f'Ignored invalid UDP sentence: {exc}')


class NmeaBridge(UdpNode):
    def __init__(self):
        super().__init__('nmea_udp_bridge')
        self.frame_id = self.declare_parameter('frame_id', 'gps').value
        self.publisher = self.create_publisher(Sentence, '/nmea_sentence', 20)

    def handle(self, sentence: str):
        for line in sentence.splitlines():
            if line:
                msg = Sentence()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                msg.sentence = line
                self.publisher.publish(msg)


class AisParser(UdpNode):
    def __init__(self):
        super().__init__('ais_parser')
        self.publisher = self.create_publisher(AIS, '/ais', 20)

    def handle(self, sentence: str):
        try:
            from pyais import decode
            value = decode(sentence)
        except Exception as exc:
            self.get_logger().warning(f'AIS decode failed: {exc}')
            return
        msg = AIS(); msg.header.stamp = self.get_clock().now().to_msg()
        msg.message_type = int(getattr(value, 'msg_type', 0) or 0)
        msg.repeat_indicator = int(getattr(value, 'repeat', getattr(value, 'repeat_indicator', 0)) or 0)
        msg.mmsi = int(getattr(value, 'mmsi', 0) or 0)
        msg.nav_status = int(getattr(value, 'status', getattr(value, 'nav_status', 0)) or 0)
        msg.turn_rate = float(getattr(value, 'turn', 0.0) or 0.0)
        msg.maneuver = int(getattr(value, 'maneuver', 0) or 0)
        msg.speed = float(getattr(value, 'speed', 0.0) or 0.0)
        msg.accuracy = bool(getattr(value, 'accuracy', False))
        msg.longitude = float(getattr(value, 'lon', 0.0) or 0.0); msg.latitude = float(getattr(value, 'lat', 0.0) or 0.0)
        msg.course = float(getattr(value, 'course', 0.0) or 0.0); msg.heading = int(getattr(value, 'heading', 0) or 0)
        msg.second = int(getattr(value, 'second', 0) or 0); msg.raim = bool(getattr(value, 'raim', False)); msg.radio = int(getattr(value, 'radio', 0) or 0)
        self.publisher.publish(msg)


class ArpaParser(UdpNode):
    def __init__(self):
        super().__init__('arpa_parser')
        self.publisher = self.create_publisher(RadarTarget, '/radar_target', 10)
        self.targets = {}
        self.timeout_sec = self.declare_parameter('target_timeout_sec', 5.0).value
        self.create_timer(1.0, self.publish_nearest)

    def handle(self, sentence: str):
        if not sentence.startswith('$RATTM'):
            return
        fields = sentence.split(',')
        if len(fields) < 14:
            raise ValueError('short RATTM message')
        target_id, distance, bearing = int(fields[1]), float(fields[2]), float(fields[3])
        self.targets[target_id] = (distance, bearing, time.monotonic())

    def publish_nearest(self):
        now = time.monotonic()
        self.targets = {key: value for key, value in self.targets.items() if now - value[2] <= self.timeout_sec}
        if self.targets:
            _, bearing, _ = min(self.targets.values(), key=lambda value: value[0])
            msg = RadarTarget(); msg.header.stamp = self.get_clock().now().to_msg(); msg.bearing_deg = bearing % 360.0
            self.publisher.publish(msg)


def spin(factory):
    rclpy.init(); node = factory()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
def ais_main(): spin(AisParser)
def arpa_main(): spin(ArpaParser)
def nmea_main(): spin(NmeaBridge)
