#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

class Decompressor(Node):
    def __init__(self):
        super().__init__('image_decompressor'); self.bridge=CvBridge()
        for index in range(1, 5):
            self.create_subscription(CompressedImage, f'/compressedimg{index}', lambda msg, i=index: self.convert(msg, i), 10)
        self.publishers=[self.create_publisher(Image, f'/img{i}', 10) for i in range(1,5)]
    def convert(self, msg, index):
        try:
            image=self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8'); output=self.bridge.cv2_to_imgmsg(image, encoding='bgr8'); output.header=msg.header; self.publishers[index-1].publish(output)
        except Exception as exc: self.get_logger().warning(f'JPEG decode failed: {exc}')
def main():
    rclpy.init(); node=Decompressor()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
