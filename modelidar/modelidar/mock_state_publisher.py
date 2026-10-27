#!/usr/bin/env python3
"""Fake robot motion for RViz, without hardware: the robot drives on a circle."""

import math

from geometry_msgs.msg import Quaternion, TransformStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw):
    """Return the quaternion of a rotation of `yaw` radians around z."""
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


class MockStatePublisher(Node):
    """
    Publish the wheel joint states and the odom -> base_footprint transform.

    Only the two virtual wheel joints are published: robot_state_publisher derives the
    four real wheels from them (mimic joints of the URDF).
    """

    def __init__(self):
        super().__init__('state_publisher')
        self.circle_radius = self.declare_parameter('circle_radius', 2.0).value
        self.linear_speed = self.declare_parameter('linear_speed', 0.3).value  # m/s
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.05).value
        self.wheel_separation = self.declare_parameter('wheel_separation', 0.24).value
        prefix = self.declare_parameter('prefix', '').value
        rate = self.declare_parameter('rate', 30.0).value  # Hz

        self.joint_names = [
            prefix + 'virtual_left_wheel_joint',
            prefix + 'virtual_right_wheel_joint',
        ]
        self.odom_frame = prefix + 'odom'
        self.base_frame = prefix + 'base_footprint'

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.start = self.get_clock().now()
        self.timer = self.create_timer(1.0 / rate, self.publish_state)
        self.get_logger().info(
            f'Driving on a circle of {self.circle_radius} m at {self.linear_speed} m/s')

    def publish_state(self):
        """Publish the state of the robot at the current time."""
        now = self.get_clock().now()
        t = (now - self.start).nanoseconds * 1e-9
        stamp = now.to_msg()

        # Counterclockwise: the left wheel is inside the circle.
        theta = self.linear_speed * t / self.circle_radius  # angle on the circle
        half = self.wheel_separation / 2
        left_path = self.circle_radius - half  # radius of each wheel's path
        right_path = self.circle_radius + half

        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = self.joint_names
        joint_state.position = [
            left_path * theta / self.wheel_radius,
            right_path * theta / self.wheel_radius,
        ]
        omega = self.linear_speed / self.circle_radius
        joint_state.velocity = [
            left_path * omega / self.wheel_radius,
            right_path * omega / self.wheel_radius,
        ]

        odom = TransformStamped()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.transform.translation.x = self.circle_radius * math.cos(theta)
        odom.transform.translation.y = self.circle_radius * math.sin(theta)
        odom.transform.rotation = yaw_to_quaternion(theta + math.pi / 2)

        self.joint_pub.publish(joint_state)
        self.tf_broadcaster.sendTransform(odom)


def main(args=None):
    rclpy.init(args=args)
    node = MockStatePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
