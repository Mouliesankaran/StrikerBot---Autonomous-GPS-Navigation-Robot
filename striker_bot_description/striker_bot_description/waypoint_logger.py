import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import yaml
import sys

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')
        self.subscription = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.current_gps = None
        self.waypoints = []
        self.get_logger().info("Logger ready. Press Enter to log, 'q' to save and exit.")

    def gps_callback(self, msg):
        self.current_gps = msg

    def save_point(self):
        if self.current_gps:
            wp = {'latitude': self.current_gps.latitude, 'longitude': self.current_gps.longitude}
            self.waypoints.append(wp)
            print(f"Logged {len(self.waypoints)}: {wp}")
        else:
            print("Waiting for GPS fix...")

def main():
    rclpy.init()
    logger = WaypointLogger()
    
    import threading
    thread = threading.Thread(target=rclpy.spin, args=(logger,), daemon=True)
    thread.start()

    try:
        while True:
            cmd = input("Hit Enter to log (q to quit): ")
            if cmd.lower() == 'q':
                break
            logger.save_point()
    finally:
        with open('gps_waypoints.yaml', 'w') as f:
            yaml.dump({'waypoints': logger.waypoints}, f)
        print("File saved. You can now run follow_waypoints.")
        rclpy.shutdown()

if __name__ == '__main__':
    main()