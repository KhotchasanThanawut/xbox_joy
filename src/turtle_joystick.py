#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry  # For receiving robot position

class TurtleJoystick(Node):
    def __init__(self):
        super().__init__('turtle_joystick')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Joy, '/joy', self.joystick_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.base_speed_linear = 1.0
        self.base_speed_angular = 1.0
        self.deadzone = 0.1
        self.speed_multiplier = 1.0

        self.saved_positions = []  # list to store saved waypoints
        self.current_position = (0.0, 0.0)  # Current robot position

    def odom_callback(self, msg):
        # Extract position from the Odometry message
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def joystick_callback(self, msg):
        twist = Twist()

        # Read joystick axes (left stick)
        forward_back = msg.axes[1]
        left_right = msg.axes[0]

        # Apply deadzone
        if abs(forward_back) < self.deadzone:
            forward_back = 0.0
        if abs(left_right) < self.deadzone:
            left_right = 0.0

        # Check buttons for speed modifiers (X = faster, Y = slower)
        # X button = 2, Y button = 3
        if len(msg.buttons) > 3:
            if msg.buttons[2]:  # X pressed -> faster (1.5x speed)
                self.speed_multiplier = 1.5
            elif msg.buttons[3]:  # Y pressed -> slower (0.5x speed)
                self.speed_multiplier = 0.5
            else:
                self.speed_multiplier = 1.0

        # Check if A button (button 0) pressed to save position
        if len(msg.buttons) > 0 and msg.buttons[0]:  # A button
            # Save current robot position as a waypoint
            waypoint = self.current_position
            self.saved_positions.append(waypoint)
            self.get_logger().info(f"Saved waypoint: x={waypoint[0]:.2f}, y={waypoint[1]:.2f}")

        # Calculate speeds with multiplier
        twist.linear.x = forward_back * self.base_speed_linear * self.speed_multiplier
        twist.angular.z = -left_right * self.base_speed_angular * self.speed_multiplier

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleJoystick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
