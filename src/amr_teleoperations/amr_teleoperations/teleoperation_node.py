#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from amr_msgs.msg import KeyBoard
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Teleoperation(Node):
    def _init_(self):
        super()._init_("teleoperation")

        # --- Publisher (cmd_vel) ---
        self.publisher_ = self.create_publisher(
            Twist,
            "cmd_vel",
            10
        )

        # --- Subscriber (keypress) ---
        self.subscription = self.create_subscription(
            KeyBoard,
            "keypress",
            self.listener_callback,
            10
        )
        self.subscription  # evitar warning

        # --- QoS para /scan (sensor data) ---
        # Normalmente los sensores van en BEST_EFFORT (menos latencia, tolera pérdidas)
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # --- Subscriber (/scan) desde ld08_driver ---
        self.scan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            scan_qos
        )
        self.scan_subscription  # evitar warning

        # --- Estado de seguridad (última distancia delante) ---
        self.front_min_dist = float("inf")

        # Parámetros de seguridad
        self.stop_distance = 0.35          # metros: si hay algo más cerca, se bloquea avance
        self.front_sector_deg = 20.0       # grados a izquierda/derecha del frente para vigilar

    def scan_callback(self, scan: LaserScan) -> None:
        """
        Guarda la distancia mínima en un sector frontal.
        """
        if not scan.ranges:
            self.front_min_dist = float("inf")
            return

        # Sector frontal alrededor de 0 rad (frente)
        half = math.radians(self.front_sector_deg)

        # Convertimos ángulos a índices del array ranges
        i0 = self._angle_to_index(0.0 - half, scan)
        i1 = self._angle_to_index(0.0 + half, scan)

        if i0 > i1:
            i0, i1 = i1, i0

        # Filtrar valores inválidos (0, inf, nan) y quedarnos con el mínimo útil
        valid = []
        for r in scan.ranges[i0:i1 + 1]:
            if r is None:
                continue
            if math.isfinite(r) and r > scan.range_min and r < scan.range_max:
                valid.append(r)

        self.front_min_dist = min(valid) if valid else float("inf")

    def _angle_to_index(self, angle_rad: float, scan: LaserScan) -> int:
        """
        Convierte un ángulo (rad) en un índice válido de scan.ranges.
        """
        # Clampeamos el ángulo al rango del sensor
        a = max(scan.angle_min, min(scan.angle_max, angle_rad))
        idx = int(round((a - scan.angle_min) / scan.angle_increment))
        idx = max(0, min(len(scan.ranges) - 1, idx))
        return idx

    def listener_callback(self, msg: KeyBoard) -> None:
        key = msg.Key
        twist = Twist()

        # --- Parada de emergencia ---
        if key == "space":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.get_logger().warn("PARADA DE EMERGENCIA (SPACE)")
            return

        # --- Mapeo teclado -> velocidades ---
        if key == "w":
            # Antes de avanzar, comprobamos colisión frontal
            if self.front_min_dist < self.stop_distance:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().warn(
                    f"Bloqueado avance: obstáculo a {self.front_min_dist:.2f} m (< {self.stop_distance:.2f} m)"
                )
            else:
                twist.linear.x = 0.2

        elif key == "s":
            twist.linear.x = -0.2

        elif key == "a":
            twist.angular.z = 0.8

        elif key == "d":
            twist.angular.z = -0.8

        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher_.publish(twist)
        self.get_logger().info(
            f"Key: {key} | front_min={self.front_min_dist:.2f} -> cmd_vel (lin.x={twist.linear.x}, ang.z={twist.angular.z})"
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    teleoperation = Teleoperation()
    rclpy.spin(teleoperation)

    teleoperation.destroy_node()
    rclpy.shutdown()


if __name__ == "_main_":
    main()