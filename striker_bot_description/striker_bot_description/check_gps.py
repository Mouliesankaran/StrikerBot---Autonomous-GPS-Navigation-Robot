#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class CheckGPS(Node):
    def __init__(self):
        super().__init__('check_gps')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10)
        self.get_logger().info("GPS Checker Started - Waiting for GPS data...")
        
    def gps_callback(self, msg):
        self.get_logger().info(f"GPS Received: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}")

def main():
    rclpy.init()
    node = CheckGPS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()