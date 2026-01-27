#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from amr_msgs.msg import KeyBoard
from geometry_msgs.msg import Twist


class Teleoperation(Node):
    def _init_(self):
        super()._init_("teleoperation")

        # Publisher a cmd_vel (igual estructura que MinimalPublisher)
        self.publisher_ = self.create_publisher(
            Twist,
            "cmd_vel",
            10
        )

        # Subscriber a keypress (igual estructura que MinimalSubscriber)
        self.subscription = self.create_subscription(
            KeyBoard,
            "keypress",
            self.listener_callback,
            10
        )
        self.subscription  # para evitar warning de unused variable

    def listener_callback(self, msg: KeyBoard) -> None:
        key = msg.Key

        twist = Twist()

        if key == "w":
            twist.linear.x = 0.1
        elif key == "s":
            twist.linear.x = -0.1
        elif key == "a":
            twist.angular.z = 0.1
        elif key == "d":
            twist.angular.z = -0.1

        # PARADA DE EMERGENCIA
        elif key == "space":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().warn("PARADA DE EMERGENCIA (SPACE)")

        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher_.publish(twist)


def main(args=None) -> None:
    rclpy.init(args=args)

    teleoperation = Teleoperation()
    rclpy.spin(teleoperation)

    teleoperation.destroy_node()
    rclpy.shutdown()


if __name__ == "_main_":
    main()