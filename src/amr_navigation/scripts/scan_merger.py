#!/usr/bin/env python3
"""Fusión de los dos LiDAR de 270° en un único escaneo de 360°.

  /lidar_front/scan ─┐
                     ├─► transformación a base_footprint ─► filtro de huella ─► /scan
  /lidar_rear/scan  ─┘                                     (puntos del propio robot)

El escaneo resultante es "virtual": está centrado en base_footprint y tiene
`bins` sectores angulares (por defecto 0.25°); en cada sector se queda el punto
más cercano. Lo consumen slam_toolbox, AMCL y los costmaps de Nav2.

Los puntos dentro del rectángulo de la huella del robot (más un margen) se
descartan: así se eliminan las autodetecciones del chasis en los extremos del
campo de visión de cada LiDAR.
"""
import math
from array import array

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class ScanMerger(Node):
    def __init__(self):
        super().__init__('scan_merger')
        self.declare_parameter('front_topic', '/lidar_front/scan')
        self.declare_parameter('rear_topic', '/lidar_rear/scan')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('bins', 1440)
        self.declare_parameter('range_min', 0.3)
        self.declare_parameter('range_max', 25.0)
        # Semilados del rectángulo de autofiltrado [m] (huella 1.2 x 0.8 + margen)
        self.declare_parameter('self_filter_x', 0.66)
        self.declare_parameter('self_filter_y', 0.46)
        self.declare_parameter('max_pair_dt', 0.1)

        gp = lambda n: self.get_parameter(n).value
        self.target = gp('target_frame')
        self.bins = int(gp('bins'))
        self.rmin, self.rmax = float(gp('range_min')), float(gp('range_max'))
        self.fx, self.fy = float(gp('self_filter_x')), float(gp('self_filter_y'))
        self.max_dt = float(gp('max_pair_dt'))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.mounts = {}          # frame -> (x, y, yaw) del LiDAR en target_frame
        self.rear = None          # (stamp_sec, puntos Nx2)

        self.pub = self.create_publisher(LaserScan, gp('output_topic'), 10)
        self.create_subscription(LaserScan, gp('rear_topic'), self._on_rear, qos_profile_sensor_data)
        self.create_subscription(LaserScan, gp('front_topic'), self._on_front, qos_profile_sensor_data)
        self.get_logger().info(f'Fusionando LiDAR → {gp("output_topic")} ({self.bins} sectores, '
                               f'autofiltro ±{self.fx:.2f} x ±{self.fy:.2f} m)')

    # ------------------------------------------------------------------
    def _mount(self, frame):
        """Pose 2D del LiDAR en target_frame (transformada estática, se cachea)."""
        if frame not in self.mounts:
            try:
                t = self.tf_buffer.lookup_transform(self.target, frame, Time())
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'Esperando TF {self.target} ← {frame}: {e}',
                                       throttle_duration_sec=5.0)
                return None
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.mounts[frame] = (t.transform.translation.x, t.transform.translation.y, yaw)
        return self.mounts[frame]

    def _points(self, msg):
        mount = self._mount(msg.header.frame_id)
        if mount is None:
            return None
        r = np.frombuffer(msg.ranges, dtype=np.float32) if isinstance(msg.ranges, array) \
            else np.asarray(msg.ranges, dtype=np.float32)
        a = msg.angle_min + np.arange(r.size, dtype=np.float32) * msg.angle_increment
        ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        r, a = r[ok], a[ok] + mount[2]
        return np.stack([mount[0] + r * np.cos(a), mount[1] + r * np.sin(a)], axis=1)

    @staticmethod
    def _sec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def _on_rear(self, msg):
        pts = self._points(msg)
        if pts is not None:
            self.rear = (self._sec(msg.header.stamp), pts)

    def _on_front(self, msg):
        pts = self._points(msg)
        if pts is None:
            return
        if self.rear is not None and abs(self._sec(msg.header.stamp) - self.rear[0]) <= self.max_dt:
            pts = np.vstack([pts, self.rear[1]])

        # Autofiltro: descartar puntos sobre el propio robot
        inside = (np.abs(pts[:, 0]) < self.fx) & (np.abs(pts[:, 1]) < self.fy)
        pts = pts[~inside]

        rng = np.hypot(pts[:, 0], pts[:, 1])
        keep = (rng >= self.rmin) & (rng <= self.rmax)
        pts, rng = pts[keep], rng[keep]
        ang = np.arctan2(pts[:, 1], pts[:, 0])

        inc = 2.0 * math.pi / self.bins
        idx = np.clip(((ang + math.pi) / inc).astype(np.int64), 0, self.bins - 1)
        ranges = np.full(self.bins, np.inf, dtype=np.float32)
        np.minimum.at(ranges, idx, rng.astype(np.float32))

        out = LaserScan()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.target
        out.angle_min = -math.pi
        out.angle_max = math.pi - inc
        out.angle_increment = inc
        out.time_increment = 0.0
        out.scan_time = msg.scan_time
        out.range_min = self.rmin
        out.range_max = self.rmax
        out.ranges = array('f', ranges.tobytes())   # sin conversión elemento a elemento
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(ScanMerger())


if __name__ == '__main__':
    main()
