#!/usr/bin/env python3
"""Comanda la altura de elevación del AMR (0.0 .. 1.0 m).

Envía a lift_controller una trayectoria sobre lift_joint a velocidad nominal
constante (por defecto 0.05 m/s, adecuada para 1000 kg de carga).

Uso:
    ros2 run amr_gazebo lift_cmd.py 1.0            # subir 1.0 m
    ros2 run amr_gazebo lift_cmd.py 0.0            # bajar
    ros2 run amr_gazebo lift_cmd.py 0.5 --speed 0.03
"""
import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ['lift_joint']
MAX_HEIGHT = 1.0


class LiftCommander(Node):
    def __init__(self):
        super().__init__('lift_cmd')
        self.client = ActionClient(self, FollowJointTrajectory,
                                   '/lift_controller/follow_joint_trajectory')
        self.current = None
        self.create_subscription(JointState, '/joint_states', self._js, 10)

    def _js(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in JOINTS):
            self.current = pos[JOINTS[0]]

    def run(self, height, speed):
        while self.current is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('lift_controller no disponible')
            return False

        duration = max(abs(height - self.current) / speed, 1.0)
        point = JointTrajectoryPoint()
        point.positions = [height]
        point.velocities = [0.0]
        point.time_from_start = Duration(sec=int(duration),
                                         nanosec=int((duration % 1) * 1e9))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        goal.trajectory.points = [point]

        self.get_logger().info(
            f'Elevación {self.current:.3f} m → {height:.3f} m en {duration:.1f} s')
        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle.accepted:
            self.get_logger().error('Objetivo rechazado')
            return False
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        code = result.result().result.error_code
        rclpy.spin_once(self, timeout_sec=0.2)
        ok = code == FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info(
            f'{"OK" if ok else f"ERROR (código {code})"} — altura actual {self.current:.3f} m')
        return ok


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('height', type=float, help=f'altura de elevación [0 .. {MAX_HEIGHT}] m')
    parser.add_argument('--speed', type=float, default=0.05, help='velocidad de la plataforma [m/s]')
    args, ros_args = parser.parse_known_args()

    if not 0.0 <= args.height <= MAX_HEIGHT:
        parser.error(f'la altura debe estar entre 0 y {MAX_HEIGHT} m')

    rclpy.init(args=ros_args)
    node = LiftCommander()
    ok = node.run(args.height, args.speed)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
