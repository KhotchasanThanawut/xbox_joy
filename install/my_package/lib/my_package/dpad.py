# dpad_control.py

from geometry_msgs.msg import Twist

def handle_dpad(msg, twist, linear_speed, angular_speed):
    # Check D-pad input (axes 9 and 10 on some Xbox controllers)
    up = msg.axes[7] > 0.5
    down = msg.axes[7] < -0.5
    left = msg.axes[6] < -0.5
    right = msg.axes[6] > 0.5

    if up:
        twist.linear.x = linear_speed
        twist.angular.z = 0.0
    elif down:
        twist.linear.x = -linear_speed
        twist.angular.z = 0.0
    elif left:
        twist.linear.x = 0.0
        twist.angular.z = angular_speed
    elif right:
        twist.linear.x = 0.0
        twist.angular.z = -angular_speed

    return twist

