#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.time import Time
from lifecycle_msgs.msg import State

from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler
import math


class OdometryNode(LifecycleNode):

    def __init__(self):
        super().__init__("odometry_node")

        self._previous_msg = None
        self._publisher = None
        self._subscription = None

    # -----------------------------------------------------------
    # LIFECYCLE METHODS
    # -----------------------------------------------------------

    def on_configure(self, state: State):

        self.get_logger().info("Configuring odometry_node...")

        # Publisher NORMAL (no lifecycle publisher)
        self._publisher = self.create_publisher(
            Odometry,
            "/odometry",
            10
        )

        # Subscription created here
        self._subscription = self.create_subscription(
            Odometry,
            "/odom",
            self._odom_callback,
            10
        )

        self._previous_msg = None

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info("Activating odometry_node...")
        return super().on_activate(state)

    def on_deactivate(self, state: State):
        self.get_logger().info("Deactivating odometry_node...")
        return super().on_deactivate(state)

    # -----------------------------------------------------------
    # CALLBACK
    # -----------------------------------------------------------

    def _odom_callback(self, msg: Odometry):

        if self._previous_msg is None:
            self._previous_msg = msg
            return

        # --- Time ---
        t_now = Time.from_msg(msg.header.stamp)
        t_prev = Time.from_msg(self._previous_msg.header.stamp)
        dt = (t_now - t_prev).nanoseconds / 1e9

        if dt <= 1e-6:
            return

        # --- Position ---
        dx = msg.pose.pose.position.x - self._previous_msg.pose.pose.position.x
        dy = msg.pose.pose.position.y - self._previous_msg.pose.pose.position.y

        linear_velocity = math.sqrt(dx * dx + dy * dy) / dt

        # --- Orientation ---
        q_now = msg.pose.pose.orientation
        q_prev = self._previous_msg.pose.pose.orientation

        _, _, theta_now = quat2euler(
            (q_now.w, q_now.x, q_now.y, q_now.z)
        )
        _, _, theta_prev = quat2euler(
            (q_prev.w, q_prev.x, q_prev.y, q_prev.z)
        )

        dtheta = theta_now - theta_prev

        # Wrap angle
        if dtheta > math.pi:
            dtheta -= 2.0 * math.pi
        elif dtheta < -math.pi:
            dtheta += 2.0 * math.pi

        angular_velocity = dtheta / dt

        # --- Build message ---
        odom_msg = Odometry()
        odom_msg.header = msg.header
        odom_msg.pose = msg.pose
        odom_msg.twist.twist.linear.x = float(linear_velocity)
        odom_msg.twist.twist.angular.z = float(angular_velocity)

        self._publisher.publish(odom_msg)

        self._previous_msg = msg


# -----------------------------------------------------------
# MAIN
# -----------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
