#!/usr/bin/env python3
"""Solo simulación: anima el mástil telescópico visual en Gazebo.

Lee la altura real del elevador (lift_joint en /joint_states) y publica la
consigna de cada etapa telescópica (h / 4) en /amr/lift_stage_cmd, que el
puente ROS-GZ entrega a los JointPositionController de las 4 etapas.
En RViz no hace falta: robot_state_publisher resuelve los mimic.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

STAGES = 4


class LiftStageRelay(Node):
    def __init__(self):
        super().__init__('lift_stage_relay')
        self.pub = self.create_publisher(Float64, '/amr/lift_stage_cmd', 10)
        self.create_subscription(JointState, '/joint_states', self._on_js, 1)
        self.last = None
        self.height = 0.0
        self.create_timer(0.1, self._publish)      # 10 Hz es suficiente para lo visual

    def _on_js(self, msg):
        try:
            self.height = msg.position[msg.name.index('lift_joint')]
        except ValueError:
            pass

    def _publish(self):
        # Solo cuando cambia (> 1 mm) o cada ~2 s para re-sincronizar
        if self.last is None or abs(self.height - self.last[0]) > 0.001 or self.last[1] >= 20:
            self.pub.publish(Float64(data=self.height / STAGES))
            self.last = [self.height, 0]
        else:
            self.last[1] += 1


def main():
    rclpy.init()
    rclpy.spin(LiftStageRelay())


if __name__ == '__main__':
    main()
