#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class SerialMotorDriver(Node):
    def __init__(self):
        super().__init__('serial_motor_driver')
        
        # --- CONFIGURATION ---
        self.serial_port = '/dev/ttyUSB0'  # Check your port!
        self.baud_rate = 57600
        
        # Motor calibration factors
        self.linear_factor = 255.0  # Increased for better speed
        self.angular_factor = 200.0
        self.min_pwm = 30  # Minimum PWM to overcome friction
        
        # Subscribe to Teleop Command
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
            
        # Initialize Serial Connection
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
            time.sleep(2)  # Wait for Arduino reset
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            self.ser = None

    def listener_callback(self, msg):
        if not self.ser:
            return

        # 1. Get Linear and Angular speeds
        linear = msg.linear.x
        angular = msg.angular.z
        
        # 2. Differential drive calculations
        # Left and right wheel velocities
        left_speed = linear - angular
        right_speed = linear + angular
        
        # 3. Apply factors and convert to PWM
        left_pwm_raw = left_speed * self.linear_factor
        right_pwm_raw = right_speed * self.linear_factor
        
        # 4. Apply deadband and minimum PWM
        if abs(left_pwm_raw) < self.min_pwm and abs(left_pwm_raw) > 0:
            left_pwm = self.min_pwm if left_pwm_raw > 0 else -self.min_pwm
        else:
            left_pwm = int(left_pwm_raw)
            
        if abs(right_pwm_raw) < self.min_pwm and abs(right_pwm_raw) > 0:
            right_pwm = self.min_pwm if right_pwm_raw > 0 else -self.min_pwm
        else:
            right_pwm = int(right_pwm_raw)
        
        # 5. Constrain to [-255, 255]
        left_pwm = max(min(left_pwm, 255), -255)
        right_pwm = max(min(right_pwm, 255), -255)
        
        # 6. Create command and send
        command = f"{left_pwm},{right_pwm}\n"
        self.ser.write(command.encode('utf-8'))
        
        # Debug output (optional)
        # self.get_logger().info(f"Linear: {linear:.2f}, Angular: {angular:.2f} -> L: {left_pwm}, R: {right_pwm}")

    def stop(self):
        if self.ser:
            self.ser.write("0,0\n".encode('utf-8'))
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = SerialMotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
        node.get_logger().info("Motor driver stopped")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()