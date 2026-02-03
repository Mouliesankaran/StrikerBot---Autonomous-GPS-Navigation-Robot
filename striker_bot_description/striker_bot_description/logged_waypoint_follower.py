import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import yaml
import os
import sys
import time
from robot_localization.srv import FromLL
from rclpy.node import Node
from striker_bot_description.gps_utils import latLonYaw2Geopose
from geometry_msgs.msg import PoseStamped


class YamlWaypointParser:
    """
    Parse a set of gps waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        """
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml file
        """
        gepose_wps = []
        for wp in self.wps_dict["waypoints"]:
            latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp.get("yaw", 0.0)
            gepose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
        return gepose_wps


class GpsWpCommander(Node):
    """
    Class to use nav2 gps waypoint follower to follow a set of waypoints logged in a yaml file
    """

    def __init__(self, wps_file_path):
        super().__init__('gps_wp_commander')
        self.navigator = BasicNavigator()
        self.wp_parser = YamlWaypointParser(wps_file_path)
        self.localizer = self.create_client(FromLL,  '/fromLL')
        while not self.localizer.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def start_wpf(self):
        """
        Function to start the waypoint following
        """
        # Wait for Nav2 to be active
        self.get_logger().info("Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active()
        
        wps = self.wp_parser.get_wps()
        self.get_logger().info(f"Loaded {len(wps)} waypoints from file")

        wpl = []
        for i, wp in enumerate(wps):
            self.req = FromLL.Request()
            self.req.ll_point.longitude = wp.position.longitude
            self.req.ll_point.latitude = wp.position.latitude
            self.req.ll_point.altitude = wp.position.altitude

            self.get_logger().info(f"Converting waypoint {i+1}: long={wp.position.longitude:.6f}, lat={wp.position.latitude:.6f}")

            self.future = self.localizer.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)

            if self.future.result() is None:
                self.get_logger().error(f"Failed to convert waypoint {i+1}")
                continue

            self.resp = PoseStamped()
            self.resp.header.frame_id = 'map'
            self.resp.header.stamp = self.get_clock().now().to_msg()
            self.resp.pose.position = self.future.result().map_point

            self.get_logger().info(f"Converted to: x={self.future.result().map_point.x:.2f}, y={self.future.result().map_point.y:.2f}")
            
            self.resp.pose.orientation = wp.orientation
            wpl.append(self.resp)

        if not wpl:
            self.get_logger().error("No waypoints to follow!")
            return

        self.get_logger().info("Starting waypoint following...")
        self.navigator.followWaypoints(wpl)

        # Wait for completion
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback and feedback.current_waypoint:
                self.get_logger().info(f"Executing waypoint {feedback.current_waypoint + 1}/{len(wpl)}")
            time.sleep(0.5)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Waypoints completed successfully!")
        else:
            self.get_logger().error(f"Waypoint following failed with result: {result}")


def main():
    rclpy.init()

    # allow to pass the waypoints file as an argument
    default_yaml_file_path = os.path.expanduser("~/gps_waypoints.yaml")
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    if not os.path.exists(yaml_file_path):
        print(f"Error: Waypoints file not found: {yaml_file_path}")
        print("Please run 'ros2 run striker_bot_description gps_waypoint_logger' first to record waypoints")
        return

    gps_wpf = GpsWpCommander(yaml_file_path)
    gps_wpf.start_wpf()
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()