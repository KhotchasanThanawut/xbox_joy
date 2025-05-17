# analog_control.py

from geometry_msgs.msg import Twist

def handle_analog(msg, twist, linear_speed, angular_speed, multiplier, deadzone):
    # Get analog stick values
    forward_back = msg.axes[1]
    left_right = msg.axes[0]

    # Apply deadzone
    if abs(forward_back) < deadzone:
        forward_back = 0.0
    if abs(left_right) < deadzone:
        left_right = 0.0

    # Calculate speeds
    twist.linear.x = forward_back * linear_speed * multiplier
    twist.angular.z = -left_right * angular_speed * multiplier

    return twist
