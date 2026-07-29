#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from message_interface.msg import PtzCtrl, RadarTarget

class PtzController(Node):
    def __init__(self):
        super().__init__('ptz_ctrl')
        self.pan = self.declare_parameter('initial_pan_deg', 135.0).value
        self.tilt = self.declare_parameter('initial_tilt_deg', 0.0).value
        self.zoom = self.declare_parameter('initial_zoom', 0).value
        self.auto_target = self.declare_parameter('auto_target', False).value
        self.publisher = self.create_publisher(PtzCtrl, '/ptz_ctrl', 10)
        self.create_subscription(RadarTarget, '/radar_target', self.target, 10)
        self.create_timer(0.5, self.publish)
    def target(self, msg):
        if self.auto_target: self.pan = msg.bearing_deg % 360.0
    def publish(self):
        msg=PtzCtrl(); msg.pan_deg=float(self.pan); msg.tilt_deg=float(min(90.0,max(0.0,self.tilt))); msg.zoom=int(self.zoom); self.publisher.publish(msg)
def main():
    rclpy.init(); node=PtzController()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
