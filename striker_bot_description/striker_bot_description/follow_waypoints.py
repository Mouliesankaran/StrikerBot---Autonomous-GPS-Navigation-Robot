import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from robot_localization.srv import FromLL
from geometry_msgs.msg import PoseStamped
import yaml
import os
import time

class GpsWaypointFollower(Node):
    def __init__(self):
        super().__init__('gps_waypoint_follower')
        self.navigator = BasicNavigator()
        
        # Create service client for GPS to map conversion
        self.client = self.create_client(FromLL, '/fromLL')
        
        # Wait for service to become available
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for /fromLL service...')
            rclpy.spin_once(self, timeout_sec=1.0)
        
        self.get_logger().info('GPS to Map conversion service available!')

    def convert_gps_to_map(self, latitude, longitude):
        """Convert GPS coordinates to map frame using robot_localization service"""
        req = FromLL.Request()
        req.ll_point.latitude = latitude
        req.ll_point.longitude = longitude
        req.ll_point.altitude = 0.0
        
        future = self.client.call_async(req)
        
        # Wait for the service call to complete
        start_time = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                try:
                    response = future.result()
                    if response:
                        self.get_logger().info(f"GPS ({latitude}, {longitude}) -> Map ({response.map_point.x:.2f}, {response.map_point.y:.2f})")
                        return response.map_point
                    else:
                        self.get_logger().error("Service call returned None")
                        return None
                except Exception as e:
                    self.get_logger().error(f'Service call failed: {e}')
                    return None
            
            if time.time() - start_time > 10.0:
                self.get_logger().error("Service call timeout")
                return None
        
        return None

    def run(self):
        # Load waypoints from file
        file_path = 'gps_waypoints.yaml'
        if not os.path.exists(file_path):
            self.get_logger().error(f"File {file_path} not found!")
            self.get_logger().info(f"Current directory: {os.getcwd()}")
            return

        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
            gps_points = data['waypoints']

        self.get_logger().info(f"Converting {len(gps_points)} GPS points to map frame...")
        
        # Convert GPS waypoints to map frame
        route = []
        for idx, gps in enumerate(gps_points):
            self.get_logger().info(f"Converting waypoint {idx+1}: {gps}")
            
            map_point = self.convert_gps_to_map(gps['latitude'], gps['longitude'])
            
            if map_point is None:
                self.get_logger().error(f"Failed to convert waypoint {idx+1}")
                continue
            
            # Create PoseStamped message
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x = map_point.x
            p.pose.position.y = map_point.y
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0  # Face forward
            
            route.append(p)
            self.get_logger().info(f"Waypoint {idx+1} -> Map: ({map_point.x:.2f}, {map_point.y:.2f})")

        if not route:
            self.get_logger().error("No valid waypoints to follow!")
            return

        # Set initial pose (critical for GPS navigation)
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = route[0].pose.position.x
        initial_pose.pose.position.y = route[0].pose.position.y
        initial_pose.pose.position.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f"Setting initial pose to: ({initial_pose.pose.position.x:.2f}, {initial_pose.pose.position.y:.2f})")
        self.navigator.setInitialPose(initial_pose)
        
        # Wait for Nav2 to become active
        self.get_logger().info("Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active()
        
        self.get_logger().info("Nav2 is active! Starting mission...")
        self.get_logger().info("3-second pause at each waypoint.")

        # Send the waypoints
        self.navigator.followWaypoints(route)

        # Monitor execution
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 10 == 0:
                self.get_logger().info(
                    f"Executing waypoint {feedback.current_waypoint + 1}/{len(route)}"
                )
            time.sleep(0.1)

        # Get the result
        result = self.navigator.getResult()
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Mission Succeeded!")
        elif result == TaskResult.CANCELED:
            self.get_logger().warn("Mission was canceled!")
        elif result == TaskResult.FAILED:
            self.get_logger().error("Mission failed!")
        else:
            self.get_logger().error(f"Unknown mission result: {result}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = GpsWaypointFollower()
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Waypoint follower interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Error in waypoint follower: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()