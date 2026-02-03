#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import math

class FakeGPSPublisher(Node):
    def __init__(self):
        super().__init__('fake_gps_publisher')
        
        # Create publisher
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        # Timer to publish at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_gps)
        
        # Base GPS coordinates (simulated)
        self.base_lat = 45.0  # Latitude in degrees
        self.base_lon = -122.0  # Longitude in degrees
        self.counter = 0
        
        self.get_logger().info("Fake GPS Publisher Started")
    
    def publish_gps(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'
        
        # Add some movement to simulate different waypoints
        offset = self.counter * 0.0001  # Small offset in degrees
        
        msg.latitude = self.base_lat + offset
        msg.longitude = self.base_lon + offset
        msg.altitude = 100.0  # Altitude in meters
        
        # Set status to FIX (2 = FIX, 0 = NO_FIX)
        msg.status.status = 2  # STATUS_FIX
        msg.status.service = 1  # SERVICE_GPS
        
        # Set some covariance (optional)
        msg.position_covariance[0] = 0.1
        msg.position_covariance[4] = 0.1
        msg.position_covariance[8] = 0.1
        msg.position_covariance_type = 2  # COVARIANCE_TYPE_APPROXIMATED
        
        self.publisher_.publish(msg)
        
        self.get_logger().info(f"Published GPS: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}")
        
        self.counter += 1
        if self.counter > 10:
            self.counter = 0

def main(args=None):
    rclpy.init(args=args)
    node = FakeGPSPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()