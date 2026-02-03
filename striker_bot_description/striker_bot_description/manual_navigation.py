#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time

def main():
    rclpy.init()
    
    # Create navigator
    navigator = BasicNavigator()
    
    # Wait a bit for other nodes to start
    print("Waiting 5 seconds for system to initialize...")
    time.sleep(5)
    
    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    
    print("Setting initial pose...")
    navigator.setInitialPose(initial_pose)
    
    # Wait for Nav2 to be active
    print("Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()
    
    print("Nav2 is active!")
    
    # Define waypoints (in meters relative to start)
    waypoints = [
        (1.0, 0.0, 0.0),    # Move 1 meter forward
        (1.0, 1.0, 0.0),    # Move 1 meter right
        (0.0, 1.0, 0.0),    # Move 1 meter back
        (0.0, 0.0, 0.0),    # Return to start
    ]
    
    # Create poses
    poses = []
    for x, y, z in waypoints:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0
        poses.append(pose)
    
    # Follow waypoints
    print(f"Starting navigation with {len(poses)} waypoints...")
    navigator.followWaypoints(poses)
    
    # Wait for completion
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        if i % 10 == 0:  # Print every 10 iterations
            print("Navigation in progress...")
        time.sleep(0.5)
    
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("✓ Navigation succeeded!")
    elif result == TaskResult.CANCELED:
        print("⚠ Navigation was canceled!")
    elif result == TaskResult.FAILED:
        print("✗ Navigation failed!")
    else:
        print(f"? Unknown navigation result: {result}")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()