import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

import message_filters
from amr_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import traceback

from amr_control.wall_follower import WallFollower


class WallFollowerNode(LifecycleNode):
    def __init__(self):
        """Wall follower node initializer."""
        super().__init__("wall_follower")

        # Parameters
        self.declare_parameter("dt", 0.1    )
        self.declare_parameter("enable_localization", False)
        self.declare_parameter("simulation", False)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles a configuring transition."""
        self.get_logger().info(f"Transitioning from '{state.label}' to 'inactive' state.")

        try:
            dt = self.get_parameter("dt").get_parameter_value().double_value
            enable_localization = (
                self.get_parameter("enable_localization").get_parameter_value().bool_value
            )
            self._simulation = self.get_parameter("simulation").get_parameter_value().bool_value

            self._wall_follower = WallFollower(
                dt,
                simulation=self._simulation,
                logger=None,
            )

            # Publishers
            # TODO: 2.10. Create the /cmd_vel velocity commands publisher (TwistStamped message).
            if self._simulation:
                self._cmd_vel_pub = self.create_publisher(
                    TwistStamped,
                    "/cmd_vel",
                    QoSProfile(depth=10),
                )
            else:
                self._cmd_vel_pub = self.create_publisher(
                    Twist,
                    "/cmd_vel",
                    QoSProfile(depth=10),
                )

            # Subscribers
            # TODO: 2.7. Synchronize _compute_commands_callback with /odometry and /scan.

            self._subscribers: list[message_filters.Subscriber] = []

            self._subscribers.append(
                message_filters.Subscriber(
                    self,
                    Odometry,
                    "/odometry",
                    qos_profile=QoSProfile(depth=10),
                )
            )

            self._subscribers.append(
                message_filters.Subscriber(
                    self,
                    LaserScan,
                    "/scan",
                    qos_profile=QoSProfile(
                        reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        history=QoSHistoryPolicy.KEEP_LAST,
                        durability=QoSDurabilityPolicy.VOLATILE,
                        depth=10,
                    ),
                )
            )

            slop = 9.0 if self._simulation else 0.15

            self._ts = message_filters.ApproximateTimeSynchronizer(
                self._subscribers,
                queue_size=10,
                slop=slop,
            )

            if self._simulation:
                self._ts.registerCallback(self._compute_commands_callback)
            else:
                self._ts.registerCallback(self._store_synced_data)

            # TODO: 4.12. Add /pose to the synced subscriptions only if localization is enabled.

            # Real robot → control timer
            if not self._simulation:
                self._last_odom = None
                self._last_scan = None
                self._timer = self.create_timer(dt, self._control_loop)

        except Exception:
            self.get_logger().error(f"{traceback.format_exc()}")
            return TransitionCallbackReturn.ERROR

        return super().on_configure(state)

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Transitioning from '{state.label}' to 'active' state.")
        return super().on_activate(state)

    def _store_synced_data(self, odom_msg: Odometry, scan_msg: LaserScan):
        self._last_odom = odom_msg
        self._last_scan = scan_msg

    def _control_loop(self):
        if self._last_odom is None or self._last_scan is None:
            return

        z_v = self._last_odom.twist.twist.linear.x
        z_w = self._last_odom.twist.twist.angular.z
        z_scan = list(self._last_scan.ranges)

        v, w = self._wall_follower.compute_commands(z_scan, z_v, z_w)
        self._publish_velocity_commands(v, w)

    def _compute_commands_callback(
        self, odom_msg: Odometry, scan_msg: LaserScan, pose_msg: PoseStamped = PoseStamped()
    ):
        if not pose_msg.localized:
            # TODO: 2.8. Parse the odometry from the Odometry message (i.e., read z_v and z_w).
            z_v = odom_msg.twist.twist.linear.x
            z_w = odom_msg.twist.twist.angular.z

            # TODO: 2.9. Parse LiDAR measurements from the LaserScan message (i.e., read z_scan).
            z_scan = list(scan_msg.ranges)

            v, w = self._wall_follower.compute_commands(z_scan, z_v, z_w)
            self._publish_velocity_commands(v, w)

    def _publish_velocity_commands(self, v: float, w: float) -> None:
        # TODO: 2.11. Complete the function body with your code (i.e., replace the pass statement).

        if self._simulation:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.linear.x = float(v)
            msg.angular.z = float(-w)

        else:
            msg = Twist()
            msg.linear.x = float(v)
            msg.angular.z = float(-w)  # invertir signo en robot real

        self._cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    wall_follower_node = WallFollowerNode()

    try:
        rclpy.spin(wall_follower_node)
    except KeyboardInterrupt:
        pass

    wall_follower_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
