#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sshkeyboard import listen_keyboard

from amr_msgs.msg import KeyBoard


class KeyboardNode(Node):
    def _init_(self):
        super()._init_("keyboard_node")

        # Publisher (igual que en MinimalPublisher)
        self._publisher = self.create_publisher(
            KeyBoard,
            "keypress",
            10
        )

        self.get_logger().info("KeyboardNode iniciado. Escuchando teclado.")

        # En lugar de create_timer(), arrancamos sshkeyboard
        listen_keyboard(
            on_press=self._on_key_pressed,
            on_release=None,
            sequential=True
        )

    def _on_key_pressed(self, key) -> None:
        # Callback llamado por sshkeyboard
        msg = KeyBoard()
        msg.Key = str(key)
        self._publisher.publish(msg)
        self.get_logger().info(f"Publishing: {msg.Key}")


def main(args=None) -> None:
    rclpy.init(args=args)

    keyboard_node = KeyboardNode()
    rclpy.spin(keyboard_node)

    keyboard_node.destroy_node()
    rclpy.shutdown()


if __name__ == "_main_":
    main()