#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from math import atan, copysign

class CmdVelToAckermann(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann')

        # Declare and get parameters
        self.declare_parameter('wheelbase', 0.325)
        self.declare_parameter('min_steer', 0.15)
        self.declare_parameter('max_steer', 0.85)

        self.L = self.get_parameter('wheelbase').value
        self.min_steer = self.get_parameter('min_steer').value
        self.max_steer = self.get_parameter('max_steer').value

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.publisher = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z

        if abs(v) < 1e-5:
            steering_angle = 0.5  # neutral
        else:
            # Convert angular velocity to steering angle
            angle = atan(self.L * omega / v)

            # Map steering angle to range [0.15, 0.85]
            # Normalize to [-max_angle, +max_angle] and map to [min_steer, max_steer]
            # Assuming max_angle = atan(L * max_omega / min_speed), you can tune this
            angle = max(self.min_steer, min(angle, self.max_steer))
            steering_angle = angle

        ack_msg = AckermannDriveStamped()
        ack_msg.drive.speed = v
        ack_msg.drive.steering_angle = steering_angle
        self.publisher.publish(ack_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToAckermann()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
