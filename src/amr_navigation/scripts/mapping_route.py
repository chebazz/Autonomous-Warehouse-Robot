#!/usr/bin/env python3
"""Recorrido automático del depósito para construir el mapa con SLAM.

Sigue una lista de puntos (frame odom, que al arrancar coincide con el mapa)
con un control proporcional holonómico, manteniendo la orientación fija.
Recorre los tres pasillos de racks (ida y vuelta → cierres de bucle), la zona
este y la zona de carga.

    ros2 run amr_navigation mapping_route.py              # 0.6 m/s
    ros2 run amr_navigation mapping_route.py --speed 0.4

Pensado para la simulación con el robot partiendo de (0, 0, 0).
"""
import argparse
import math

import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

ROUTE = [
    # Pasillo 3 (y = -2.7): ida y vuelta
    (0.0, -2.7), (-18.3, -2.7), (-2.0, -2.7),
    # Pasillo 2 (y = 3.1)
    (-2.0, 3.1), (-18.3, 3.1), (-2.0, 3.1),
    # Pasillo 1 (y = 8.9)
    (-2.0, 8.9), (-18.3, 8.9), (-2.0, 8.9),
    # Zona este (holguras ≥ 0.2 m con columnas, estaciones y equipos)
    (1.5, 8.0), (11.0, 8.0), (11.0, 4.0), (17.3, 4.0), (17.3, 0.6),
    (13.6, 0.6), (13.6, -5.3), (3.0, -5.3), (3.0, 0.0),
    # Zona de carga (frente a las estaciones P1..P4) y regreso
    (3.0, -7.2), (-18.3, -7.2), (0.0, -7.2), (0.0, 0.0),
]


class RouteFollower(Node):
    def __init__(self, speed):
        super().__init__('mapping_route',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.speed = speed
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.idx = 0
        self.done = False
        self.create_timer(0.05, self.step)

    def pose(self):
        try:
            t = self.tf.lookup_transform('odom', 'base_footprint', Time())
        except Exception:  # noqa: BLE001
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def send(self, vx=0.0, vy=0.0, wz=0.0):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_footprint'
        m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = vx, vy, wz
        self.pub.publish(m)

    def step(self):
        if self.done:
            return
        p = self.pose()
        if p is None:
            return
        x, y, yaw = p
        gx, gy = ROUTE[self.idx]
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        if dist < 0.15:
            self.get_logger().info(f'Punto {self.idx + 1}/{len(ROUTE)} ({gx:.1f}, {gy:.1f})')
            self.idx += 1
            if self.idx >= len(ROUTE):
                self.send()
                self.done = True
                self.get_logger().info('Recorrido completado')
            return
        v = min(self.speed, 0.8 * dist + 0.05)
        ex, ey = dx / dist * v, dy / dist * v
        # al marco del robot
        c, s = math.cos(yaw), math.sin(yaw)
        self.send(c * ex + s * ey, -s * ex + c * ey, max(-0.5, min(0.5, -1.5 * yaw)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.6)
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = RouteFollower(args.speed)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
