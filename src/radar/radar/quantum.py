import socket
import struct
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from message_interface.msg import Spoke

FRAME_QUANTUM_SPOKE = 0x00280003

class Quantum(Node):
    def __init__(self):
        super().__init__('quantum_radar')
        self.group = self.declare_parameter('locator_group', '224.0.0.1').value
        self.locator_port = self.declare_parameter('locator_port', 5800).value
        self.interface_ip = self.declare_parameter('interface_ip', '0.0.0.0').value
        self.model_id = self.declare_parameter('model_id', 40).value
        self.image_pub = self.create_publisher(Image, '/radar_image', 10)
        self.polar_pub = self.create_publisher(Image, '/USV/Polar', 10)
        self.spoke_pub = self.create_publisher(Spoke, '/quantum_spoke', 50)
        self.status_pub = self.create_publisher(String, '/diagnostic_status', 10)
        self.create_subscription(String, '/radar_control', self.control, 10)
        self.bridge, self.location, self.report_sock, self.command_sock = CvBridge(), None, None, socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_sock.setblocking(False); self.spokes = np.zeros((250, 256), dtype=np.uint8); self.scanning = False
        self.locator_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP); self.locator_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1); self.locator_sock.bind(('', self.locator_port)); self.locator_sock.setblocking(False)
        self.locator_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, struct.pack('4s4s', socket.inet_aton(self.group), socket.inet_aton(self.interface_ip)))
        self.create_timer(0.02, self.poll); self.create_timer(2.5, self.publish_image); self.create_timer(3.5, self.heartbeat)

    def poll(self):
        if self.location is None:
            try: data, _ = self.locator_sock.recvfrom(1024)
            except BlockingIOError: return
            if len(data) != 36: return
            fields = struct.unpack('<IIBBHIII4sI4s', data)
            if fields[4] != self.model_id: return
            data_ip, data_port = socket.inet_ntoa(fields[7]), struct.unpack('<H', fields[8][:2])[0]
            radar_ip, radar_port = socket.inet_ntoa(fields[9]), struct.unpack('<H', fields[10][:2])[0]
            self.location = (data_ip, data_port, radar_ip, radar_port); self.report_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); self.report_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1); self.report_sock.bind(('', data_port)); self.report_sock.setblocking(False); self.report_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, struct.pack('4s4s', socket.inet_aton(data_ip), socket.inet_aton(self.interface_ip))); self.get_logger().info(f'Quantum found at {radar_ip}:{radar_port}')
        else:
            try:
                while True: self.process(self.report_sock.recvfrom(2048)[0])
            except BlockingIOError: pass

    def process(self, data):
        if len(data) < 20 or struct.unpack_from('<I', data)[0] != FRAME_QUANTUM_SPOKE: return
        _, _, _, _, spokes, _, _, azimuth, _ = struct.unpack_from('<IHHHHHHHH', data)
        decoded=[]; index=20
        while index < len(data) and len(decoded) < 256:
            if data[index] == 0x5c and index + 2 < len(data): decoded.extend([data[index+2]] * data[index+1]); index += 3
            else: decoded.append(data[index]); index += 1
        if not decoded: return
        azimuth %= 250; values=np.asarray(decoded[:256], dtype=np.uint8); self.spokes[(azimuth+125)%250,:len(values)] = values
        msg=Spoke(); msg.header.stamp=self.get_clock().now().to_msg(); msg.azimuth=azimuth; msg.data=values.tolist(); self.spoke_pub.publish(msg)

    def publish_image(self):
        image=(self.spokes.astype(np.float32) / 128.0 * 255.0).clip(0,255).astype(np.uint8)
        self.polar_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding='mono8'))
        cartesian=cv2.warpPolar(image, (512,512), (256,256), 256, cv2.WARP_INVERSE_MAP); self.image_pub.publish(self.bridge.cv2_to_imgmsg(cartesian, encoding='mono8')); self.spokes.fill(0)

    def send(self, payload):
        if self.location:
            try: self.command_sock.sendto(payload, self.location[2:])
            except OSError as exc: self.get_logger().warning(f'Radar command failed: {exc}')
    def control(self, msg):
        commands={'start_scan':bytes([0x10,0,0x28,0,1,0,0,0]), 'stop_scan':bytes([0x10,0,0x28,0,0,0,0,0])}
        if msg.data in commands: self.send(commands[msg.data]); self.scanning=msg.data=='start_scan'
    def heartbeat(self): self.status_pub.publish(String(data='Radar: Scanning' if self.scanning else 'Radar: Standby'))

def main():
    rclpy.init(); node=Quantum()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
