# StrikerBot - Autonomous GPS Navigation Robot

## Overview
StrikerBot is an autonomous four-wheeled robot simulation in ROS 2 and Gazebo, designed for precise GPS-based navigation and waypoint following. The robot features differential drive control, GPS, IMU, and camera sensors for comprehensive environmental perception.

## Features
- ✅ Four-wheel differential drive with skid steering
- ✅ Realistic GPS simulation with coordinate tracking
- ✅ IMU sensor for orientation data
- ✅ Camera sensor for visual feedback
- ✅ Precise odometry and position tracking
- ✅ Configurable control parameters for stable movement

## Hardware Specifications
- **Chassis**: 1.30m × 0.65m × 0.50m (L×W×H)
- **Wheels**: 4 × 0.15m radius, 0.10m width
- **Mass**: 150kg (chassis), 5kg per wheel
- **Sensors**: GPS, IMU, Camera
- **Drive System**: 4-wheel differential drive

## Installation & Setup

### Prerequisites
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo 11
- Python 3.10+

### 1. Clone and Build
```bash
# Create workspace
mkdir -p ~/striker_ws/src
cd ~/striker_ws/src

# Clone repository
git clone https://github.com/yourusername/striker_bot.git

# Build the package
cd ~/striker_ws
colcon build --packages-select striker_bot_description
source install/setup.bash
```

### 2. Launch Simulation
```bash
# Launch Gazebo with StrikerBot
ros2 launch striker_bot_description gazebo.launch.py
```

## File Descriptions

### Key Files
- **striker_bot.urdf.xacro** - Main robot description
  - Defines robot geometry, materials, and inertial properties
  - Configures wheel physics and joint properties
  - Sets up Gazebo plugins for control and sensors
  - Includes GPS with initial coordinates (37.7749, -122.4194)

- **gazebo.launch.py** - Launch file
  - Starts Gazebo simulation with GPS-enabled world
  - Spawns the robot model
  - Launches robot_state_publisher and joint_state_publisher

- **gps_world.world** - Gazebo world configuration
  - Sets spherical coordinates for GPS simulation
  - Provides ground plane and lighting
  - Enables accurate GPS coordinate tracking

- **check_gps.py** - GPS verification script
  - Monitors GPS topic for data
  - Verifies coordinate updates during movement
  - Outputs real-time GPS position

- **controller.yaml** - ROS 2 Control configuration
  - Configures differential drive controller parameters
  - Sets wheel separation and diameter
  - Defines velocity and acceleration limits

## Operational Commands

### Basic Movement
```bash
# Move forward at 0.2 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# Stop the robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

### Monitoring
```bash
# Check GPS data
ros2 topic echo /gps/fix

# Monitor odometry
ros2 topic echo /odom

# View joint states
ros2 topic echo /joint_states

# Check camera feed
ros2 run rqt_image_view rqt_image_view
```

### GPS Verification
```bash
# Run GPS checker
ros2 run striker_bot_description check_gps

# Get single GPS reading
ros2 topic echo /gps/fix --once
```

## Calibration Results

### Movement Calibration
- Command: 0.02 m/s for 33.33 seconds
- Expected: 0.666 meters
- Actual: 0.734 meters (10.2% faster)
- Calibration Factor: 1.102

### GPS Sensitivity
- Movement: 1 meter
- Longitude Change: ~0.000009 degrees
- GPS Accuracy: Sub-meter precision

## Troubleshooting

### Common Issues
1. **Robot doesn't move**
   - Check `/cmd_vel` topic subscription
   - Verify diff_drive plugin loaded without errors
   - Increase wheel friction (mu1, mu2) in URDF

2. **GPS shows zero coordinates**
   - Ensure world has spherical coordinates
   - Check GPS plugin initialization
   - Verify launch file uses `gps_world.world`

3. **Unstable movement**
   - Reduce max_wheel_torque in URDF
   - Increase wheel damping (kd)
   - Lower acceleration limits

### Debug Commands
```bash
# List all topics
ros2 topic list

# Check plugin status
ros2 topic echo /gazebo/plugin_states

# View Gazebo logs
tail -f ~/.ros/log/latest/gzserver*.log
```

## Project Status
### ✅ Completed
- Robot URDF modeling
- Sensor integration (GPS, IMU, Camera)
- Differential drive control
- GPS coordinate system setup
- Movement calibration
- Waypoint storage system

### 🔄 In Progress
- Waypoint following algorithm
- Obstacle avoidance
- Path planning implementation
- ROS 2 navigation stack integration

### 📋 Planned
- SLAM implementation
- Multi-robot simulation
- Web interface for control
- Real-world deployment

## Contributing
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit changes (`git commit -m 'Add AmazingFeature'`)
4. Push to branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request
